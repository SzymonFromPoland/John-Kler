#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <Preferences.h>
#include <U8g2lib.h>

#define MENU_COUNT 5

extern Preferences prefs_global;
extern U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2;

void IRAM_ATTR IRint();
void handleIR();
void saveParams();
void loadParams();  
void startIRTask(uint8_t pin);


#endif
