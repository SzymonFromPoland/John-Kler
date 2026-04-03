#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <Preferences.h>
#include <U8g2lib.h>

#define MENU_COUNT 4

extern bool started;
extern bool hold_led;
extern int mode;
extern int menu;
extern bool selected;
extern int selectedOpt;
extern bool doCalibrate;

extern float Kp;
extern float Kd;
extern float yawKp;
extern float yawKd;

extern Preferences prefs_global;
extern U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;

void IRAM_ATTR IRint();
void handleIR();
void startIRTask(uint8_t pin);

#endif
