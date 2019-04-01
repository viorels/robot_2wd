#pragma once

#define LEFT 0
#define RIGHT 1
#define FWD 1
#define BACK 0

#define SAMPLE_TIME 50

void motors_setup();
void motors_update();

void set_speed(float left, float right);
void set_direction(float angle);

// average speed between 2 wheels
float get_speed();

// get direction as measured by encoder
float get_direction();
