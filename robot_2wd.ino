#include <NewPing.h>
#include "robot_pins.h"

#define MAX_SONAR_DISTANCE 200  // can't do more than 450-500

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE);

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);
}

#define LEFT 0
#define RIGHT 1
#define FWD 0
#define BACK 1

int wheel_pins[][2] = {
  {LEFT_FWD_PIN, RIGHT_FWD_PIN},
  {LEFT_BACK_PIN, RIGHT_BACK_PIN}
};

void move_wheel(int wheel, int dir) {
  digitalWrite(wheel_pins[dir][wheel], HIGH);
}

void stop_bot() {
  digitalWrite(LEFT_FWD_PIN, LOW);
  digitalWrite(RIGHT_FWD_PIN, LOW);
  digitalWrite(LEFT_BACK_PIN, LOW);
  digitalWrite(RIGHT_BACK_PIN, LOW);
}

void move_bot(int dir) {    // direction is FWD or BACK
  stop_bot();               // first cancel all other commands

  move_wheel(LEFT, dir);
  move_wheel(RIGHT, dir);
}

void rotate_bot(int dir) {  // direction is LEFT or RIGHT
  stop_bot();               // first cancel all other commands so we can rotate in place

  if (dir == LEFT) {
    move_wheel(LEFT, BACK);
    move_wheel(RIGHT, FWD);
  }
  else if (dir == RIGHT) {
    move_wheel(LEFT, FWD);
    move_wheel(RIGHT, BACK);
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

  rotate_bot(dir);
  delay(angle / 360 * milis_per_rotation);
  stop_bot();
}

bool moving = false;
bool obstacle = false;

void loop() {
  int distance = sonar.ping_cm();
  if (distance > 0)  // if distance is 0 then nothing is in range OR it's touching an obstacle
    Serial.println(distance);
  obstacle = distance != 0 && distance < 10;

  if (!moving) {
    if (!obstacle) {
      move_bot(FWD);
      moving = true;
    }
  }
  else {  // bot is moving
    if (obstacle) {
      stop_bot();
      rotate_bot_angle(-180);
      moving = false;
    }
  }

  delay(50);
}
