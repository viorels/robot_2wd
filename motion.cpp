#define __ASSERT_USE_STDERR
#include <assert.h>

#include <PID_v1.h>
#include <Arduino.h>
#include "robot_config.h"
#include "motion.h"

// speed PID
double motor_speed[2] = {0.0, 0.0};
double motor_power[2] = {0.0, 0.0};  // 0-1 range
double target_speed[2] = {0.0, 0.0};  // in pulses/s
float target_speed_mps = 0;  // in m/s

double Kp = 0.01, Ki = 0.01, Kd = 0.0002;
double speed_pid_limits[2] = {-1.0, 1.0};
PID speed_pid[2] = {
  PID(&motor_speed[0], &motor_power[0], &target_speed[0], Kp, Ki, Kd, DIRECT),
  PID(&motor_speed[1], &motor_power[1], &target_speed[1], Kp, Ki, Kd, DIRECT)
};

// direction PID
double initial_dir = -1;  // reference for pulse differences between wheels
double robot_dir = -1;    // degrees
double target_dir = -1;   // where do we want to go
double target_dir_closest = -1;  // not normalized angle that is closest in value to current direction
double speed_diff = 0;  // +/- for each wheel to achieve target_dir

//double Kp_dir = 1, Ki_dir = 0.5, Kd_dir = 0.05;
double Kp_dir = 0.1, Ki_dir = 0.0, Kd_dir = 0.0; // Kd must be 0, else there are problems at 359 => 1 transition
double direction_pid_limits[2] = {-40, +40};
PID direction_pid(&robot_dir, &speed_diff, &target_dir_closest, Kp_dir, Ki_dir, Kd_dir, DIRECT);

volatile long pulses[2] = {0, 0};
volatile unsigned long last_pulse[2] = {0, 0};  // time for debouncing
int motors_state[2] = {0, 0};  // +1/0/-1 for each wheel, sign of power applied to the motors

void encoder_pulse(int wheel) {
  if (millis() - last_pulse[wheel] >= 2) {  // debounce 2ms
    pulses[wheel] = pulses[wheel] + motors_state[wheel];
    last_pulse[wheel] = millis();
  }
}

void encoder_left() {
  encoder_pulse(LEFT);
}

void encoder_right() {
  encoder_pulse(RIGHT);
}

static inline int8_t sign(float val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

void set_motor_power(int wheel, float power) {
  static int wheel_pins[][2] = {
    {LEFT_BACK_PIN, LEFT_FWD_PIN},
    {RIGHT_BACK_PIN, RIGHT_FWD_PIN}
  };
  static int pwm_pins[] = {LEFT_PWM_PIN, RIGHT_PWM_PIN};

  int power_sign = sign(power);
  if (power_sign != motors_state[wheel]) {
    motors_state[wheel] = power_sign;

    int pin_values[2] = {0, 0};
    if (power_sign == 1)
      pin_values[FWD] = 1;
    else if (power_sign == -1)
      pin_values[BACK] = 1;

    for (int dir = 0; dir < 2; dir++)
      digitalWrite(wheel_pins[wheel][dir], pin_values[dir]);
  }

  analogWrite(pwm_pins[wheel], abs(power) * 255);
}

float mps_to_pps(float mps) {
  // convert meters/s to encoder pulses/s
  return mps / (PI * WHEEL_DIAM) * WHEEL_ENCODER_PULSES;
}

void set_speed(float left, float right) {
  // speed for each wheel in mps
  target_speed_mps = (left + right) / 2;
  target_speed[LEFT] = mps_to_pps(left);
  target_speed[RIGHT] = mps_to_pps(right);

  // be optimistic about actual speed
  motor_speed[LEFT] = mps_to_pps(left);
  motor_speed[RIGHT] = mps_to_pps(right);
}

float normalize_angle(float angle) {
  // return an angle between 0.0 and 359.99
  float normalized = ((int)angle) % 360;
  if (normalized < 0)
    normalized += 360;
  normalized += angle - (int)angle;
  return normalized;
}

float abs_angle_diff(float a, float b) {
  if (a > b)
    return normalize_angle(a - b);
  else
    return normalize_angle(b - a);
}

void set_direction(float angle) {
  if (initial_dir == -1)
    initial_dir = angle;
  target_dir = angle;
}

float measure_direction() {
  static float value = -1;
  static float pulses_per_deg = 2.0 * WHEEL_DIST / WHEEL_DIAM * WHEEL_ENCODER_PULSES / 360;

  float alpha = 0.2;
  float measurement = normalize_angle(initial_dir + (pulses[LEFT] - pulses[RIGHT]) / pulses_per_deg);
  if (value == -1)
    value = measurement;
  else {
    value = alpha * closest_angle(measurement, value) + (1 - alpha) * value;
  }
  return normalize_angle(value);
}

float get_direction() {
  return robot_dir;
}

void measure_speed() {
  static long last_time_pulses[2] = {0, 0};

  float alpha = 0.2; // factor to tune
  for (int wheel = 0; wheel < 2; wheel++) {
    float measurement = (pulses[wheel] - last_time_pulses[wheel]) * 1000.0 / SAMPLE_TIME;
      motor_speed[wheel] = alpha * measurement + (1 - alpha) * motor_speed[wheel];
    last_time_pulses[wheel] = pulses[wheel];
  }
}

float get_speed() {
  return (motor_speed[LEFT] + motor_speed[RIGHT]) / 2;
}

float closest_angle(float angle, float reference) {
  // finds the closest angle to the reference angle, even if it means being negative or > 360
  if (angle > reference && angle - reference > 180)
    return angle - 360;
  else if (angle < reference && reference - angle > 180)
    return angle + 360;
  return angle;
}

void motors_update() {
  // called every SAMPLE_TIME ms

  robot_dir = measure_direction();
  target_dir_closest = closest_angle(target_dir, robot_dir);
  direction_pid.Compute();

  int speed_limit_pps = mps_to_pps(MAX_SPEED);
  target_speed[LEFT] = constrain(mps_to_pps(target_speed_mps) + speed_diff, -speed_limit_pps, speed_limit_pps);
  target_speed[RIGHT] = constrain(mps_to_pps(target_speed_mps) - speed_diff, -speed_limit_pps, speed_limit_pps);

  Serial.print(target_dir_closest);
  Serial.print(" - ");
  Serial.print(robot_dir);
/*
  Serial.print(" = ");
  Serial.print(target_dir_closest - robot_dir);
  Serial.print("\t");
  Serial.print(speed_diff);
*/
  for (int i = 0; i < 2; i++) {
    speed_pid[i].Compute();
  }

  measure_speed();

  Serial.print(motor_speed[LEFT]);
  Serial.print("\t");
  Serial.print(motor_power[LEFT] * 100);
  Serial.print("\t");
  Serial.print(motor_speed[RIGHT]);
  Serial.print("\t");
  Serial.print(motor_power[RIGHT] * 100);

  Serial.println("");

  for (int wheel = 0; wheel < 2; wheel++)
    set_motor_power(wheel, motor_power[wheel]);
}

void motors_setup() {
  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), encoder_left, CHANGE);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), encoder_right, CHANGE);

  for (int i = 0; i < 2; i++) {
    speed_pid[i].SetMode(AUTOMATIC);
    speed_pid[i].SetOutputLimits(speed_pid_limits[0], speed_pid_limits[1]);
    speed_pid[i].SetSampleTime(SAMPLE_TIME);
  }

  direction_pid.SetMode(AUTOMATIC);
  direction_pid.SetOutputLimits(direction_pid_limits[0], direction_pid_limits[1]);
  direction_pid.SetSampleTime(SAMPLE_TIME);
}

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  // transmit diagnostic informations through serial link.
  Serial.print("assert");
  Serial.print(" ");
  Serial.print(__sexp);
  Serial.print(" at ");
  Serial.print(__file);
  Serial.print(":");
  Serial.print(__lineno, DEC);
  Serial.print(" in ");
  Serial.println(__func);
  Serial.flush();
  // abort program execution.
  abort();
}
