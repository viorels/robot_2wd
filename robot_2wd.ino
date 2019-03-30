#include <NewPing.h>
#include <NewTone.h>
#include <PID_v1.h>
#include "robot_config.h"
#include "motion.h"
#include "timer.h"

#define MAX_SONAR_DISTANCE 200  // can't do more than 450-500
#define MAX_SONAR_TIME 50 // wait for echo and fade
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE);

unsigned long timer_sample = 0;
unsigned long timer_1s = 0;
unsigned long timer_rotate = 0;
unsigned long timer_sonar = 0;

void setup() {
  Serial.begin(115200);

  motors_setup();

  pinMode(BUZZER_PIN, OUTPUT);

  set_timer(timer_sample, SAMPLE_TIME);
  set_timer(timer_1s, 2 * MAX_SONAR_TIME);  // after first sonar ping
  set_timer(timer_sonar, MAX_SONAR_TIME);  // schedule next sonar ping

  sonar.ping_timer(echoCheck);  // async
  delay(MAX_SONAR_TIME);
}

extern double motor_power[2];

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
  bot_direction_change(angle);

  int dir;
  if (angle >= 0) {
    dir = RIGHT;
  }
  else {
    dir = LEFT;
    angle = abs(angle);
  }

  int milis_per_rotation = 1000;
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

    update_motors_pid();
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
