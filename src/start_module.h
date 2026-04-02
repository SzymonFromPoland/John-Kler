#ifndef START_MODULE_H
#define START_MODULE_H

#include <Arduino.h>
#include <RC5.h>
#include <Preferences.h>
#include <U8g2lib.h>

extern bool started;
extern bool hold_led;
extern Preferences prefs_global;
extern U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;

void IRAM_ATTR IRint();
void handleIR();
void startIRTask(uint8_t pin);

#endif
