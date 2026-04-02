#include "start_module.h"
#include "config.h"

RC5 rc5(RCV);

bool started = false;
bool hold_led = false;
volatile bool irFlag = false;
volatile unsigned char toggle, address, command;

void IRAM_ATTR IRint()
{
  if (rc5.read((unsigned char *)&toggle, (unsigned char *)&address, (unsigned char *)&command))
  {
    irFlag = true;
  }
}

void handleIR()
{
  if (irFlag)
  {
    uint8_t START, STOP;

    prefs_global.begin("robot", true);
    STOP = prefs_global.getUInt("stop_address", 0);
    START = prefs_global.getUInt("start_address", 0);
    prefs_global.end();

    if (address == 0x0B)
    {
      START = command + 1;
      STOP = command;

      prefs_global.begin("robot", false);
      prefs_global.putUInt("stop_address", STOP);
      prefs_global.putUInt("start_address", START);
      prefs_global.end();

      hold_led = true;
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_spleen8x16_me);
      u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth("PROGRAMMED")) / 2, 16);
      u8g2.print("PROGRAMMED");
      u8g2.sendBuffer();
      delay(500);
      hold_led = false;
    }
    else if (address == 0x07)
    {
      if (command == START)
        started = true;
      else if (command == STOP)
        started = false;
    }
    irFlag = false;
  }
}

void startIRTask(uint8_t pin)
{
  rc5 = RC5(pin);
  pinMode(RCV, INPUT);
  attachInterrupt(digitalPinToInterrupt(RCV), IRint, CHANGE);
}