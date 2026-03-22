#ifndef START_MODULE_H
#define START_MODULE_H

#include <Arduino.h>
#include <RC5.h>
#include <Preferences.h>

extern bool started;
extern bool hold_led;
extern Preferences prefs_global;

void startIRTask(uint8_t pin, uint8_t ledPin = -1);

#endif
