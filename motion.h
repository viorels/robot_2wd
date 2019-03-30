#pragma once

#define LEFT 0
#define RIGHT 1
#define FWD 1
#define BACK 0

#define SAMPLE_TIME 50

void motors_setup();
void move_wheel(int wheel, int dir, float speed);
void motors_power(float left, float right);
void update_wheel_power();
void bot_direction(float angle);
void bot_direction_change(float angle);
float get_direction();
float get_speed(int wheel);
void update_motors_pid();
void stop_bot();
