#pragma once

#include <Arduino.h>

void set_timer(unsigned long &timer, int duration);
bool check_timer(unsigned long &timer);
bool check_timer_repeat(unsigned long &timer, int duration);

float normalize_angle(float angle);
float abs_angle_diff(float a, float b);
float closest_angle(float angle, float reference);

extern int8_t sign(float val);

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp);
