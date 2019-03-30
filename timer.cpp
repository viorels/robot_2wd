#include <Arduino.h>
#include "timer.h"

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
