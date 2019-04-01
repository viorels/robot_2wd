#include <Arduino.h>
#include "utils.h"

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

bool check_timer_repeat(unsigned long &timer, int duration) {
  if (check_timer(timer)) {
    set_timer(timer, duration);
    return true;
  }
  
  return false;
}

float normalize_angle(float angle) {
  // return an angle between 0.0 and 359.99
  float normalized = ((int)angle) % 360;
  if (normalized < 0)
    normalized += 360;
  normalized += angle - (int)angle;
  return normalized;
}

float abs_angle_diff(float a, float b) {
  if (a > b)
    return normalize_angle(a - b);
  else
    return normalize_angle(b - a);
}

float closest_angle(float angle, float reference) {
  // finds the closest angle to the reference angle, even if it means being negative or > 360
  if (angle > reference && angle - reference > 180)
    return angle - 360;
  else if (angle < reference && reference - angle > 180)
    return angle + 360;
  return angle;
}

static inline int8_t sign(float val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
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
