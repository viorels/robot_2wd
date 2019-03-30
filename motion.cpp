#define __ASSERT_USE_STDERR
#include <assert.h>

#include <PID_v1.h>
#include <Arduino.h>
#include "robot_config.h"
#include "motion.h"

// speed PID
double motor_speed[2] = {0, 0};
double motor_power[2] = {0.5, 0.5};  // 0-1 range
double target_speed = 40;  // 20 slits and 40 change pulses per rotation

double Kp=0.01, Ki=0.01, Kd=0.0;
PID speed_pid[2] = {
  PID(&motor_speed[0], &motor_power[0], &target_speed, Kp, Ki, Kd, DIRECT),
  PID(&motor_speed[1], &motor_power[1], &target_speed, Kp, Ki, Kd, DIRECT)
};

// direction PID
double robot_dir = 0;   // degrees
double target_dir = 0;  // where do we want to go
double power_diff = 0;  // +/- for each wheel to achieve target_dir

double Kp_dir=0.01, Ki_dir=0.01, Kd_dir=0.0;
PID direction_pid(&robot_dir, &power_diff, &target_dir, Kp_dir, Ki_dir, Kd_dir, DIRECT);

volatile long pulses[2] = {0, 0};
volatile unsigned long last_pulse[2] = {0, 0};  // time for debouncing
int pulse_dir[2] = {0, 0};  // +1/0/-1 for each wheel, depending on how power is applied to the engine
int pulse_dir_delta[2] = {-1, +1};  // BACK = -1, FWD = +1

void encoder_pulse(int wheel) {
  if (millis() - last_pulse[wheel] >= 2) {  // debounce 2ms
    pulses[wheel] = pulses[wheel] + pulse_dir[wheel];
    last_pulse[wheel] = millis();
  }
}

void encoder_left() {
  encoder_pulse(LEFT);
}

void encoder_right() {
  encoder_pulse(RIGHT);
}

int pwm_pins[] = {LEFT_PWM_PIN, RIGHT_PWM_PIN};

void move_wheel(int wheel, int dir, float speed) {
  static int wheel_pins[][2] = {
    {LEFT_BACK_PIN, RIGHT_BACK_PIN},
    {LEFT_FWD_PIN, RIGHT_FWD_PIN}
  };

//  assert(speed > 0);
  if (speed > 0) {
    digitalWrite(wheel_pins[dir][wheel], HIGH);
    analogWrite(pwm_pins[wheel], speed * 255);
    pulse_dir[wheel] = pulse_dir_delta[dir];
  }
  else {
    // TODO, stop both directions and reimplement stop_bot
    pulse_dir[wheel] = 0;
  }
}

void motors_power(float left, float right) {
  motor_power[LEFT] = left;
  motor_power[RIGHT] = right;
}
  

void update_wheel_power() {
  for (int wheel = 0; wheel < 2; wheel++)
    analogWrite(pwm_pins[wheel], motor_power[wheel] * 255);
}

float normalize_angle(float angle) {
  // return an angle between 0.0 and 359.99
  float normalized = ((int)angle) % 360;
  if (normalized < 0)
    normalized += 360;
  normalized += angle - (int)angle;
  return normalized;
}

void bot_direction(float angle) {
  target_dir = angle;
}

void bot_direction_change(float angle) {
  bot_direction(normalize_angle(target_dir + angle));
}

void stop_bot() {
  digitalWrite(LEFT_FWD_PIN, LOW);
  digitalWrite(RIGHT_FWD_PIN, LOW);
  digitalWrite(LEFT_BACK_PIN, LOW);
  digitalWrite(RIGHT_BACK_PIN, LOW);
  pulse_dir[LEFT] = 0;
  pulse_dir[RIGHT] = 0;
}

float get_direction() {
  static float pulses_per_deg = 2.0 * WHEEL_DIST / WHEEL_DIAM * WHEEL_ENCODER_PULSES / 360;
  return normalize_angle((pulses[LEFT] - pulses[RIGHT]) / pulses_per_deg);
}

float get_speed(int wheel) {
  static long last_time_pulses[2] = {0, 0};
  static float value[2] = {0, 0};

  float alpha = 0.25; // factor to tune
  float measurement = (pulses[wheel] - last_time_pulses[wheel]) * 1000.0/SAMPLE_TIME;
  value[wheel] = alpha * measurement + (1-alpha) * value[wheel];
  last_time_pulses[wheel] = pulses[wheel];

  return abs(value[wheel]);
}

void update_motors_pid() {
  motor_speed[0] = get_speed(LEFT);
  motor_speed[1] = get_speed(RIGHT);
/*
  Serial.print(motor_speed[0]);
  Serial.print(" ");
  Serial.print(motor_speed[1]);
  Serial.print(" ");
*/
  for (int i=0; i<2; i++) {
    speed_pid[i].Compute();
  }

  robot_dir = get_direction();
  direction_pid.Compute();
  float motor_power_left = constrain(motor_power[LEFT] + power_diff, 0.1, 1);
  float motor_power_right = constrain(motor_power[RIGHT] - power_diff, 0.1, 1);
  motors_power(motor_power_left, motor_power_right);

  Serial.print(robot_dir);
  Serial.print(" ");
  for (int i=0; i<2; i++) {
    speed_pid[i].Compute();
    Serial.print(motor_power[i] * 100);
    Serial.print(" ");
  }
  Serial.println("");

  update_wheel_power();  
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

  for (int i=0; i<2; i++) {
    speed_pid[i].SetMode(AUTOMATIC);
    speed_pid[i].SetOutputLimits(0.1, 1.0);
    speed_pid[i].SetSampleTime(SAMPLE_TIME);
  }

  direction_pid.SetMode(AUTOMATIC);
  direction_pid.SetOutputLimits(-1, 1);
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
