#include <NewPing.h>
#include "NewTone.h"
#include "robot_pins.h"

#define MAX_SONAR_DISTANCE 200  // can't do more than 450-500
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE);

#define LEFT 0
#define RIGHT 1
#define FWD 0
#define BACK 1

unsigned long timer_1s = 0;
unsigned long timer_rotate = 0;

void set_timer(unsigned long &timer, int duration) {
  timer = millis() + duration;
}

bool check_timer(unsigned long &timer) {
  if (timer != 0 && millis() > timer) {
    timer = 0;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  int setup_duration = 30;  // wait 30ms for sonar
  set_timer(timer_1s, setup_duration);

  sonar.ping_timer(echoCheck);  // async
  delay(setup_duration);
}

int wheel_pins[][2] = {
  {LEFT_FWD_PIN, RIGHT_FWD_PIN},
  {LEFT_BACK_PIN, RIGHT_BACK_PIN}
};
int pwm_pins[] = {LEFT_PWM_PIN, RIGHT_PWM_PIN};

void move_wheel(int wheel, int dir, float speed) {
  digitalWrite(wheel_pins[dir][wheel], HIGH);
  analogWrite(pwm_pins[wheel], speed * 255);
}

void stop_bot() {
  digitalWrite(LEFT_FWD_PIN, LOW);
  digitalWrite(RIGHT_FWD_PIN, LOW);
  digitalWrite(LEFT_BACK_PIN, LOW);
  digitalWrite(RIGHT_BACK_PIN, LOW);
}

void move_bot(int dir, float speed) {
  /*  direction is FWD or BACK
   *  speed is between 0 and 1
   */

  stop_bot();               // first cancel all other commands

  move_wheel(LEFT, dir, speed);
  move_wheel(RIGHT, dir, speed);
}

void rotate_bot(int dir, float speed) {
  /*  direction is LEFT or RIGHT
      speed is between 0 and 1
  */

  stop_bot();               // first cancel all other commands so we can rotate in place

  if (dir == LEFT) {
    move_wheel(LEFT, BACK, speed);
    move_wheel(RIGHT, FWD, speed);
  }
  else if (dir == RIGHT) {
    move_wheel(LEFT, FWD, speed);
    move_wheel(RIGHT, BACK, speed);
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

  rotate_bot(dir, 1);
}

unsigned long last_buzz = 0;

#define NOTE_C3  131

void sonar_buzz(int distance) {
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
    Serial.println(distance);
  }
  // Don't do anything here!
}

#define STOPPED 0
#define MOVING 1
#define ROTATING 2

int state = STOPPED;
bool obstacle = false;

void loop() {
  sonar.ping_timer(echoCheck);  // async

  if (distance > 0) {  // if distance is 0 then nothing is in range OR it's touching an obstacle
    sonar_buzz(distance);
  }
  obstacle = distance != 0 && distance < 10;

  if (check_timer(timer_1s)) {
    set_timer(timer_1s, 1000);
    // execute every second
  }

  // stop rotation at specified time
  if (check_timer(timer_rotate)) {
    if (state == ROTATING) {
      stop_bot();
      state = STOPPED;
    }
  }

  switch (state) {
    case STOPPED:
      if (!obstacle) {
        move_bot(FWD, 0.7);
        state = MOVING;
      }
      break;

    case MOVING:
      if (obstacle) {
        stop_bot();
        rotate_bot_angle(-180);
        state = ROTATING;
      }
      break;

    case ROTATING:
      break;
  }

  distance = 0;  // clear distance in case the obstacle is no longer there

  delay(30);
}
