#include "controller.h"
#include "config.h"
#include <IRrecv.h>
#include <IRutils.h>

IRrecv irrecv(RCV);
decode_results irResults;

struct IRMap
{
  uint8_t code;
  const char *name;
};

IRMap irMap[] = {
    {0xA2, "A"},
    {0x62, "B"},
    {0xE2, "C"},
    {0x22, "D"},
    {0x02, "UP"},
    {0xC2, "E"},
    {0xE0, "LEFT"},
    {0xA8, "OK"},
    {0x90, "RIGHT"},
    {0x68, "0"},
    {0x98, "DOWN"},
    {0xB0, "F"},
    {0x30, "1"},
    {0x18, "2"},
    {0x7A, "3"},
    {0x10, "4"},
    {0x38, "5"},
    {0x5A, "6"},
    {0x42, "7"},
    {0x4A, "8"},
    {0x52, "9"}};

float stepKp = 10.0f;
float stepKd = 1.0f;
float Kp = 800.0f;
float Kd = 65.0f;

float yawStepKp = 0.1f;
float yawStepKd = 0.0001f;
float yawKp = 0.8f;
float yawKd = 0.0005f;

bool started = false;
bool hold_led = false;
bool doCalibrate = false;
int mode = 1;
int menu = 0;
bool selected = false;
int selectedOpt = 0;

void saveParams()
{
  prefs_global.begin("robot", false);
  prefs_global.putFloat("Kp", Kp);
  prefs_global.putFloat("Kd", Kd);
  prefs_global.putFloat("yawKp", yawKp);
  prefs_global.putFloat("yawKd", yawKd);
  prefs_global.end();
}

void loadParams()
{
  prefs_global.begin("robot", false);
  Kp = prefs_global.getFloat("Kp", 800.0f);
  Kd = prefs_global.getFloat("Kd", 65.0f);
  yawKp = prefs_global.getFloat("yawKp", 0.8f);
  yawKd = prefs_global.getFloat("yawKd", 0.0005f);
  prefs_global.end();
}

void startIRTask(uint8_t pin)
{
  irrecv.enableIRIn();
  loadParams();
}

void handleIR()
{
  if (!irrecv.decode(&irResults))
    return;

  uint8_t START, STOP;
  prefs_global.begin("robot", false);
  STOP = prefs_global.getUInt("stop_address", 0);
  START = prefs_global.getUInt("start_address", 0);

  switch (irResults.decode_type)
  {
  case RC5:
  {
    uint8_t address = (irResults.value >> 6) & 0x1F;
    uint8_t command = irResults.value & 0x3F;
    uint8_t toggle = (irResults.value >> 11) & 0x01;

    if (address == 0x0B)
    {
      START = command + 1;
      STOP = command;
      prefs_global.putUInt("stop_address", STOP);
      prefs_global.putUInt("start_address", START);

      hold_led = true;
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_spleen8x16_me);
      u8g2.setCursor(
          (u8g2.getDisplayWidth() - u8g2.getStrWidth("PROGRAMMED")) / 2,
          16);
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
    break;
  }

  case NEC:
  {
    uint8_t address = (irResults.value >> 24) & 0xFF;
    uint8_t command = (irResults.value >> 8) & 0xFF;
    bool repeat = irResults.repeat;

    if (!repeat)
    {
      for (IRMap &m : irMap)
      {
        if (m.code == command)
        {

          if (strcmp(m.name, "A") == 0)
          {
            mode = 1;
          }
          else if (strcmp(m.name, "B") == 0)
          {
            mode = 2;
          }
          else if (strcmp(m.name, "C") == 0)
          {
            mode = 3;
          }
          else if (strcmp(m.name, "E") == 0)
          {
            started = true;
          }
          else if (strcmp(m.name, "D") == 0)
          {
            started = false;
          }
          else if (strcmp(m.name, "F") == 0)
          {
            doCalibrate = true;
          }
          if (strcmp(m.name, "OK") == 0)
          {
            selected = !selected;
          }
          else if (strcmp(m.name, "RIGHT") == 0)
          {
            if (selected)
            {
              if (menu == 2)
              {
                if (selectedOpt == 0)
                {
                  Kp += stepKp;
                  saveParams();
                }
                if (selectedOpt == 1)
                {
                  Kd += stepKd;
                  saveParams();
                }
              }
              if (menu == 3)
              {
                if (selectedOpt == 0)
                {
                  yawKp += yawStepKp;
                  saveParams();
                }
                if (selectedOpt == 1)
                {
                  yawKd += yawStepKd;
                  saveParams();
                }
              }
            }
            else
            {
              menu++;
              if (menu >= MENU_COUNT)
                menu = 0;
            }
          }

          else if (strcmp(m.name, "LEFT") == 0)
          {
            if (selected)
            {
              if (menu == 2)
              {
                if (selectedOpt == 0)
                {
                  Kp -= stepKp;
                  saveParams();
                }
                if (selectedOpt == 1)
                {
                  Kd -= stepKd;
                  saveParams();
                }
              }
              if (menu == 3)
              {
                if (selectedOpt == 0)
                {
                  yawKp -= yawStepKp;
                  saveParams();
                }
                if (selectedOpt == 1)
                {
                  yawKd -= yawStepKd;
                  saveParams();
                }
              }
            }
            else
            {
              menu--;
              if (menu < 0)
                menu = MENU_COUNT - 1;
            }
          }
          else if (strcmp(m.name, "UP") == 0)
          {
            if (!selected && (menu == 2 || menu == 3))
            {
              selectedOpt++;
              if (selectedOpt > 1)
                selectedOpt = 0;
            }
          }
          else if (strcmp(m.name, "DOWN") == 0)
          {
            if (!selected && (menu == 2 || menu == 3))
            {
              selectedOpt--;
              if (selectedOpt < 0)
                selectedOpt = 1;
            }
          }

          break;
        }
      }
    }

    break;
  }

  default:
    break;
  }

  prefs_global.end();
  irrecv.resume();
}