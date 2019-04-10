#pragma once

#define LEFT 0
#define RIGHT 1
#define FWD 1
#define BACK 0

#define SAMPLE_TIME 20
#define LARGE_PULSE_INTERVAL 100  // 0.1s, 1/4 rps

void motors_setup();
void motors_update();

void set_speed(float target_speed_mps);
float get_speed();  // average speed between 2 wheels
float get_distance();   // in meters

void set_direction(float angle);
float get_direction();  // get direction as measured by encoder
float get_target_direction();
