#define __ASSERT_USE_STDERR
#include <assert.h>
#include "utils.h"

#include <PID_v2.h>
#include <Arduino.h>
#include "RunningAverage.h"
#include "robot_config.h"
#include "motion.h"

// speed PID
double motors_power = 0.0;    // -1 - +1 range
double motors_power_change = 0.0;
double motors_speed = 0.0;    // pulses/s, average between 2 wheels
double target_speed = 0.0;    // in pulses/s
float target_speed_mps = 0;   // in m/s
RunningAverage pulse_avg[2] = {RunningAverage(10), RunningAverage(10)};

double Kp = 0.001, Ki = 0.000, Kd = 0.00015;
double speed_pid_limits[2] = {-1.0, 1.0};
PID speed_pid(&motors_speed, &motors_power_change, &target_speed, Kp, Ki, Kd, DIRECT);

// direction PID
double initial_dir = -1;  // reference for pulse differences between wheels
double robot_dir = -1;    // degrees
double target_dir = -1;   // where do we want to go
double target_dir_closest = -1;  // not normalized angle that is closest in value to current direction
double power_diff = 0;    // +/- for each wheel to achieve target_dir

double Kp_dir = 0.005, Ki_dir = 0.001, Kd_dir = 0.0; // Kd must be 0, else there are problems at 359 => 1 transition
double direction_pid_limits[2] = {-1, +1};
PID direction_pid(&robot_dir, &power_diff, &target_dir_closest, Kp_dir, Ki_dir, Kd_dir, DIRECT);

volatile long pulses[2] = {0, 0};
volatile unsigned long last_pulse[2][2] = {   // time for debouncing and interval start
  {millis(), millis()},
  {millis(), millis()}};
int motors_state[2] = {0, 0};  // +1/0/-1 for each wheel, sign of power applied to the motors

void static inline encoder_pulse(int wheel, int encoder_pin) {
  int state = digitalRead(encoder_pin);

  unsigned long now = millis();

  if (now - last_pulse[wheel][state] >= 2) {  // debounce 2ms
    pulses[wheel] = pulses[wheel] + motors_state[wheel];
    float pulse_interval = now - last_pulse[wheel][state];

    float signed_pulse_interval = motors_state[wheel] * pulse_interval;
    if (motors_state[wheel] == 0 || signed_pulse_interval > LARGE_PULSE_INTERVAL) {
       signed_pulse_interval = LARGE_PULSE_INTERVAL;  // long interval to signal engine is stopped (but wheel is NOT!)
    }

    // at ~1rps there is a ~2.5Hz wheel wobbling
//    Serial.print(signed_pulse_interval);

    pulse_avg[wheel].addValue(signed_pulse_interval);  // TODO: volatile?

    last_pulse[wheel][state] = now;
  }
}

void encoder_left() {
  encoder_pulse(LEFT, LEFT_ENCODER_PIN);
}

void encoder_right() {
  encoder_pulse(RIGHT, RIGHT_ENCODER_PIN);
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

  if (!isclose(target_speed, 0)) {
    speed_pid.SetMode(AUTOMATIC);

    // be optimistic about actual speed
    pulse_avg[LEFT].addValue(1000.0 / target_speed);
    pulse_avg[RIGHT].addValue(1000.0 / target_speed);
  }
  else {
    speed_pid.SetMode(MANUAL);
  }
}

void set_direction(float angle) {
  if (initial_dir == -1)
    initial_dir = angle;
  target_dir = angle;
}

float get_distance() {
  return PI * WHEEL_DIAM * ((pulses[LEFT] + pulses[RIGHT]) / 2) / WHEEL_ENCODER_PULSES;
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
  float speed[2];
  for (int wheel = 0; wheel < 2; wheel++) {
    if (millis() - last_pulse[wheel][HIGH] > LARGE_PULSE_INTERVAL) {   // consider it stopped after 100ms
      pulse_avg[wheel].addValue(2 * LARGE_PULSE_INTERVAL);
    }

    speed[wheel] = 1000.0 / pulse_avg[wheel].getAverage();

    if (abs(speed[wheel]) < (1000 / LARGE_PULSE_INTERVAL))
      speed[wheel] = 0;
  }

  motors_speed = (speed[LEFT] + speed[RIGHT]) / 2.0;
}

float get_speed() {
  return motors_speed;
}

void motors_update() {
  // called every SAMPLE_TIME ms

  measure_speed();
  float power[2];
  if (speed_pid.GetMode() == AUTOMATIC) {
    speed_pid.Compute();
    motors_power = constrain(motors_power + motors_power_change, -1.0, +1.0);
    power[LEFT] = motors_power;
    power[RIGHT] = motors_power;
  }
  else {
    power[LEFT] = 0;
    power[RIGHT] = 0;
  }

/*
  Serial.print(motors_speed);
  Serial.print("\t");
  Serial.print(motors_power * 100);
  Serial.print("\t");
  Serial.print(target_speed);
*/

  measure_direction();
  target_dir_closest = closest_angle(target_dir, robot_dir);
  if (direction_pid.GetMode() == AUTOMATIC) {
    direction_pid.Compute();
    int speed_limit_pps = mps_to_pps(MAX_SPEED);
    power[LEFT] = constrain(power[LEFT] + power_diff, -1, 1);
    power[RIGHT] = constrain(power[RIGHT] - power_diff, -1, 1);
  }

//  Serial.print(target_dir_closest);
//  Serial.print(" - ");
//  Serial.print(robot_dir);

//  Serial.print(" = ");
//  Serial.print(target_dir_closest - robot_dir);
//  Serial.print("\t");
//  Serial.print(power_diff * 100);

  Serial.println("");

  for (int wheel = 0; wheel < 2; wheel++)
    set_motor_power(wheel, power[wheel]);
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

  for (int wheel = 0; wheel < 2; wheel++) {
    pulse_avg[wheel].clear();
    pulse_avg[wheel].addValue(LARGE_PULSE_INTERVAL);
  }

  speed_pid.SetMode(AUTOMATIC);
  speed_pid.SetOutputLimits(speed_pid_limits[0], speed_pid_limits[1]);
  speed_pid.SetSampleTime(SAMPLE_TIME);

  direction_pid.SetMode(AUTOMATIC);
  direction_pid.SetOutputLimits(direction_pid_limits[0], direction_pid_limits[1]);
  direction_pid.SetSampleTime(SAMPLE_TIME);
}
