#define __ASSERT_USE_STDERR
#include <assert.h>
#include "utils.h"

#include <PID_v1.h>
#include <Arduino.h>
#include "robot_config.h"
#include "motion.h"

// speed PID
double motors_power = 0.0;    // -1 - +1 range
double motors_speed = 0.0;    // pulses/s, average between 2 wheels
double target_speed = 0.0;    // in pulses/s
float target_speed_mps = 0;   // in m/s

double Kp = 0.02, Ki = 0.0, Kd = 0.0;
double speed_pid_limits[2] = {-1.0, 1.0};
PID speed_pid(&motors_speed, &motors_power, &target_speed, Kp, Ki, Kd, DIRECT);

// direction PID
double initial_dir = -1;  // reference for pulse differences between wheels
double robot_dir = -1;    // degrees
double target_dir = -1;   // where do we want to go
double target_dir_closest = -1;  // not normalized angle that is closest in value to current direction
double speed_diff = 0;  // +/- for each wheel to achieve target_dir

//double Kp_dir = 1, Ki_dir = 0.5, Kd_dir = 0.05;
double Kp_dir = 0.2, Ki_dir = 1, Kd_dir = 0.0; // Kd must be 0, else there are problems at 359 => 1 transition
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

void set_speed(float target_speed_mps) {
  target_speed = mps_to_pps(target_speed_mps);

  // be optimistic about actual speed
  motors_speed = target_speed;
}

void set_direction(float angle) {
  if (initial_dir == -1)
    initial_dir = angle;
  target_dir = angle;
}

void measure_direction() {
  static float pulses_per_deg = 2.0 * WHEEL_DIST / WHEEL_DIAM * WHEEL_ENCODER_PULSES / 360;

  float alpha = 0.5;  // TODO: this should not be needed
  float measurement = normalize_angle(initial_dir + (pulses[LEFT] - pulses[RIGHT]) / pulses_per_deg);
  robot_dir = normalize_angle(alpha * closest_angle(measurement, robot_dir) + (1 - alpha) * robot_dir);
}

float get_direction() {
  return robot_dir;
}

float get_target_direction() {
  return target_dir;
}

void measure_speed() {
  static long last_time_pulses[2] = {0, 0};

  long new_pulses = 0;
  for (int wheel = 0; wheel < 2; wheel++) {
    new_pulses += pulses[wheel] - last_time_pulses[wheel];
    last_time_pulses[wheel] = pulses[wheel];
  }
  new_pulses /= 2.0;  // average the 2 wheels
  double measurement = new_pulses * 1000.0 / SAMPLE_TIME;

  float alpha = 0.1; // factor to tune
  motors_speed = alpha * measurement + (1 - alpha) * motors_speed;
}

float get_speed() {
  return motors_speed;
}

void motors_update() {
  // called every SAMPLE_TIME ms

  measure_speed();
  speed_pid.Compute();

  Serial.print(motors_speed);
  Serial.print("\t");
  Serial.print(motors_power * 100);
  Serial.print("\t");
  Serial.print(target_speed);

/*
  measure_direction();
  target_dir_closest = closest_angle(target_dir, robot_dir);
  if (direction_pid.GetMode() == AUTOMATIC && direction_pid.Compute()) {
    int speed_limit_pps = mps_to_pps(MAX_SPEED);
    target_speed[LEFT] = constrain(mps_to_pps(target_speed_mps) + speed_diff, -speed_limit_pps, speed_limit_pps);
    target_speed[RIGHT] = constrain(mps_to_pps(target_speed_mps) - speed_diff, -speed_limit_pps, speed_limit_pps);
  }

  Serial.print(target_dir_closest);
  Serial.print(" - ");
  Serial.print(robot_dir);
//  Serial.print(" = ");
//  Serial.print(target_dir_closest - robot_dir);
  Serial.print("\t");
  Serial.print(speed_diff * 10);

*/
  Serial.println("");

  for (int wheel = 0; wheel < 2; wheel++)
//    set_motor_power(wheel, motors_power);
    set_motor_power(wheel, 0.5);
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

  speed_pid.SetMode(AUTOMATIC);
  speed_pid.SetOutputLimits(speed_pid_limits[0], speed_pid_limits[1]);
  speed_pid.SetSampleTime(SAMPLE_TIME);

  direction_pid.SetMode(AUTOMATIC);
  direction_pid.SetOutputLimits(direction_pid_limits[0], direction_pid_limits[1]);
  direction_pid.SetSampleTime(SAMPLE_TIME);
}
