#define __ASSERT_USE_STDERR

#include <assert.h>
#include <NewPing.h>
#include "NewTone.h"
#include <PID_v1.h>
#include "robot_pins.h"

#define MAX_SONAR_DISTANCE 200  // can't do more than 450-500
#define MAX_SONAR_TIME 50 // wait for echo and fade
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE);

#define LEFT 0
#define RIGHT 1
#define FWD 1
#define BACK 0

unsigned long timer_sample = 0;
unsigned long timer_1s = 0;
unsigned long timer_rotate = 0;
unsigned long timer_sonar = 0;

void set_timer(unsigned long &timer, int duration) {
  // trigger after duration (in millis)
  timer = millis() + duration;
}

bool check_timer(unsigned long &timer) {
  // check if timer is enabled and already passed
  if (timer != 0 && millis() > timer) {
    timer = 0;
    return true;
  }
  return false;
}

// speed PID
double motor_speed[2] = {0, 0};
double motor_power[2] = {0.5, 0.5};  // 0-1 range
double target_speed = 40;  // 20 slits and 40 change pulses per rotation

#define SAMPLE_TIME 50

//Specify the links and initial tuning parameters
double Kp=0.01, Ki=0.01, Kd=0.0;
PID speed_pid[2] = {
  PID(&motor_speed[0], &motor_power[0], &target_speed, Kp, Ki, Kd, DIRECT),
  PID(&motor_speed[1], &motor_power[1], &target_speed, Kp, Ki, Kd, DIRECT)
};

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), encoder_left, CHANGE);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), encoder_right, CHANGE);

  set_timer(timer_sample, SAMPLE_TIME);
  set_timer(timer_1s, 2 * MAX_SONAR_TIME);  // after first sonar ping
  set_timer(timer_sonar, MAX_SONAR_TIME);  // schedule next sonar ping

  for (int i=0; i<2; i++) {
    speed_pid[i].SetMode(AUTOMATIC);
    speed_pid[i].SetOutputLimits(0.1, 1.0);
    speed_pid[i].SetSampleTime(SAMPLE_TIME);
  }

  sonar.ping_timer(echoCheck);  // async
  delay(MAX_SONAR_TIME);
}

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

  assert(speed > 0);
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

void update_wheel_power() {
  for (int wheel = 0; wheel < 2; wheel++)
    analogWrite(pwm_pins[wheel], motor_power[wheel] * 255);
}

void stop_bot() {
  digitalWrite(LEFT_FWD_PIN, LOW);
  digitalWrite(RIGHT_FWD_PIN, LOW);
  digitalWrite(LEFT_BACK_PIN, LOW);
  digitalWrite(RIGHT_BACK_PIN, LOW);
  pulse_dir[LEFT] = 0;
  pulse_dir[RIGHT] = 0;
}

void move_bot(int dir) {
  /*  direction is FWD or BACK
   *  speed is between 0 and 1
   */

  stop_bot();               // first cancel all other commands

  move_wheel(LEFT, dir, motor_power[LEFT]);
  move_wheel(RIGHT, dir, motor_power[RIGHT]);
}

void rotate_bot(int dir) {
  /*  direction is LEFT or RIGHT
      speed is between 0 and 1
  */

  stop_bot();               // first cancel all other commands so we can rotate in place

  if (dir == LEFT) {
    move_wheel(LEFT, BACK, motor_power[LEFT]);
    move_wheel(RIGHT, FWD, motor_power[RIGHT]);
  }
  else if (dir == RIGHT) {
    move_wheel(LEFT, FWD, motor_power[LEFT]);
    move_wheel(RIGHT, BACK, motor_power[RIGHT]);
  }
}

void rotate_bot_angle(float angle) {  // positive for clockwise, negative for counterclockwise
  // this needs encoders for decent precision, otherwise it depends on how charged is the battery
  int milis_per_rotation = 900;       // just an estimation for an almost full battery...

  int dir;
  if (angle >= 0) {
    dir = RIGHT;
  }
  else {
    dir = LEFT;
    angle = abs(angle);
  }

  int duration = angle / 360 * milis_per_rotation;
  set_timer(timer_rotate, duration);

  rotate_bot(dir);
}

#define NOTE_C3  131

void sonar_buzz(int distance) {
  static unsigned long last_buzz = 0;
  int buzz_delay = 100 + distance * 10;
  int buzz_tone = NOTE_C3 * (4 - 3 * (float)distance / MAX_SONAR_DISTANCE);
  int buzz_duration = 25 + distance;
  if (millis() - last_buzz > buzz_delay) {
    NewTone(BUZZER_PIN, buzz_tone, buzz_duration);  // TODO: use TimerFreeTone to free up timer1?
    last_buzz = millis();
  }
}

int distance = 0;

void echoCheck() {  // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) {  // This is how you check to see if the ping was received.
    distance = sonar.ping_result / US_ROUNDTRIP_CM;  // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
//    Serial.println(distance);
  }
  // Don't do anything here!
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

// bot states
#define STOPPED 0
#define MOVING 1
#define ROTATING 2

void loop() {
  static int bot_state = STOPPED;
  static bool obstacle = false;

  long loop_start = millis();

  if (check_timer(timer_sonar)) {
    set_timer(timer_sonar, MAX_SONAR_TIME);
    distance = 0;  // clear distance in case the obstacle is no longer there
    sonar.ping_timer(echoCheck);  // async
  }

  if (distance > 0) {  // if distance is 0 then nothing is in range OR it's touching an obstacle
    sonar_buzz(distance);
  }
  obstacle = distance != 0 && distance < 10;

  if (check_timer(timer_sample)) {
    set_timer(timer_sample, SAMPLE_TIME);

    motor_speed[0] = get_speed(LEFT);
    motor_speed[1] = get_speed(RIGHT);
    Serial.print(motor_speed[0]);
    Serial.print(" ");
    Serial.print(motor_speed[1]);
    Serial.print(" ");

    for (int i=0; i<2; i++) {
      if (speed_pid[i].Compute()) {
        update_wheel_power();
      }
      Serial.print(motor_power[i] * 100);
      Serial.print(" ");
    }
    Serial.println("");
  }


  if (check_timer(timer_1s)) {
    // execute every second
    set_timer(timer_1s, 1000);

  }

  // stop rotation at specified time
  if (check_timer(timer_rotate)) {
    if (bot_state == ROTATING) {
      stop_bot();
      bot_state = STOPPED;
    }
  }

  switch (bot_state) {
    case STOPPED:
      if (!obstacle) {
        move_bot(FWD);
        bot_state = MOVING;
      }
      break;

    case MOVING:
      if (obstacle) {
        stop_bot();
        rotate_bot_angle(-180);
        bot_state = ROTATING;
      }
      break;

    case ROTATING:
      break;
  }
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
