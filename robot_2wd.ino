#include <NewPing.h>
#include <NewTone.h>
#include <PID_v1.h>
#include "robot_config.h"
#include "motion.h"
#include "utils.h"

#define MAX_SONAR_DISTANCE 200  // can't do more than 450-500
#define MAX_SONAR_TIME 50 // wait for echo and fade
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE);

unsigned long timer_sample = 0;
unsigned long timer_1s = 0;
unsigned long timer_sonar = 0;

float speed = MAX_SPEED / 3;  // at least 1/4
float direction = 90;

void setup() {
  Serial.begin(115200);

  set_direction(direction);
  motors_setup();

  pinMode(BUZZER_PIN, OUTPUT);

  set_timer(timer_sample, SAMPLE_TIME);
  set_timer(timer_1s, 2 * MAX_SONAR_TIME);  // after first sonar ping
  set_timer(timer_sonar, MAX_SONAR_TIME);  // schedule next sonar ping

  sonar.ping_timer(echoCheck);  // async
  delay(MAX_SONAR_TIME);
}

void move_bot(int dir) {
  /*  direction is FWD or BACK
   */
  static int dir_sign[2] = {-1, +1};
  set_speed(dir_sign[dir] * speed);
}

void rotate_bot(float angle) {
  // angle is positive for clockwise, negative for counterclockwise
  direction = normalize_angle(get_target_direction() + angle);
  set_direction(direction);
  int angle_sign = sign(angle);
//  set_speed(angle_sign * speed, -angle_sign * speed);
}

bool rotation_finished() {
  return abs_angle_diff(get_direction(), direction) < 15;
}

void stop_bot() {
  set_speed(0);
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

    motors_update();
  }

  if (check_timer(timer_1s)) {
    // execute every second
    set_timer(timer_1s, 1000);

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
        rotate_bot(180);
        bot_state = ROTATING;
      }
      break;

    case ROTATING:
      if (rotation_finished()) {
        bot_state = STOPPED;
      }

      break;
  }
}
