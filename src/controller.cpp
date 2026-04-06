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

uint8_t address;
uint8_t command;

void saveParams()
{
  prefs_global.begin("robot", false);
  prefs_global.putFloat("Kp", Kp);
  prefs_global.putFloat("Kd", Kd);
  prefs_global.putFloat("yawKp", yawKp);
  prefs_global.putFloat("yawKd", yawKd);
  prefs_global.putFloat("speed", speed);
  prefs_global.putFloat("max_speed", max_speed);
  prefs_global.putFloat("rot_speed", rot_speed);
  prefs_global.putFloat("ramp_up_gain", ramp_up_step);
  prefs_global.putFloat("flag_threshold", flag_threshold);
  prefs_global.putFloat("sens_threshold", threshold);
  prefs_global.putFloat("detect_flag", (float)detect_flag);
  prefs_global.putFloat("arch_angle", arch_angle);
  prefs_global.putFloat("arch_speed", arch_speed);
  prefs_global.putFloat("arch_time", arch_time);
  prefs_global.putFloat("mode", (float)mode);
  prefs_global.end();
}

void loadParams()
{
  prefs_global.begin("robot", false);
  Kp = prefs_global.getFloat("Kp", 8.0f);
  Kd = prefs_global.getFloat("Kd", 0.65f);
  yawKp = prefs_global.getFloat("yawKp", 0.8f);
  yawKd = prefs_global.getFloat("yawKd", 0.0005f);
  speed = prefs_global.getFloat("speed", 50.0f);
  max_speed = prefs_global.getFloat("max_speed", 50.0f);
  rot_speed = prefs_global.getFloat("rot_speed", 60.0f);
  ramp_up_step = prefs_global.getFloat("ramp_up_gain", 1.0f);
  flag_threshold = prefs_global.getFloat("flag_threshold", 500000.0f);
  threshold = prefs_global.getFloat("sens_threshold", 300.0f);
  detect_flag = (bool)prefs_global.getFloat("detect_flag", 0.0f);
  arch_angle = prefs_global.getFloat("arch_angle", 50.0f);
  arch_speed = prefs_global.getFloat("arch_speed", 100.0f);
  arch_time = prefs_global.getFloat("arch_time", 1000.0f);
  mode = (int)prefs_global.getFloat("mode", 1.0f);

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

    bool repeat = irResults.repeat;

    static uint8_t lastAddress = 0xFF;
    static uint8_t lastCommand = 0xFF;

    if (!repeat)
    {
      address = (irResults.value >> 24) & 0xFF;
      command = (irResults.value >> 8) & 0xFF;
      lastAddress = address;
      lastCommand = command;
    }
    else
    {
      address = lastAddress;
      command = lastCommand;
    }

    for (IRMap &m : irMap)
    {
      if (m.code == command)
      {

        if (strcmp(m.name, "E") == 0)
        {
          started = true;
        }
        else if (strcmp(m.name, "D") == 0)
        {
          started = false;
        }

        if (!started)
        {
          if (strcmp(m.name, "F") == 0 && !repeat)
          {
            doCalibrate = true;
          }
          if (strcmp(m.name, "A") == 0 && !repeat)
          {
            detect_flag = !detect_flag;
          }
          else if (strcmp(m.name, "OK") == 0 && !repeat)
          {
            if (menu != 0)
              selected = !selected;
            if (menu == 0)
              test_servo = true;
          }
          else if (strcmp(m.name, "RIGHT") == 0)
          {
            if (selected)
            {
              if (menu == 1)
              {
                targetYaw += 5;
                if (targetYaw > 180)
                  targetYaw = -180;
              }
              if (menu == 2)
              {
                if (selectedOpt == 0)
                {
                  speed += speedStep;
                  speed = constrain(speed, 0, 100);
                }
                if (selectedOpt == 1)
                {
                  max_speed += speedStep;
                  max_speed = constrain(max_speed, 0, 100);
                }
                if (selectedOpt == 2)
                {
                  rot_speed += speedStep;
                  rot_speed = constrain(rot_speed, 0, 100);
                }
                if (selectedOpt == 3)
                {
                  ramp_up_step += 0.05;
                  ramp_up_step = constrain(ramp_up_step, 0, 100);
                }
              }
              if (menu == 3)
              {
                if (selectedOpt == 0)
                {
                  Kp += stepKp;
                }
                if (selectedOpt == 1)
                {
                  Kd += stepKd;
                }
              }
              if (menu == 4)
              {
                if (selectedOpt == 0)
                {
                  yawKp += yawStepKp;
                }
                if (selectedOpt == 1)
                {
                  yawKd += yawStepKd;
                }
              }
              if (menu == 5)
              {
                if (selectedOpt == 0)
                {
                  flag_threshold += 10000;
                }
                if (selectedOpt == 1)
                {
                  threshold += 10.0f;
                  threshold = constrain(threshold, 0.0f, 1000.0f);
                }
              }
              if (menu == 6)
              {
                if (selectedOpt == 0)
                {
                  arch_speed += 5.0f;
                  arch_speed = constrain(arch_speed, 0.0f, 100.0f);
                }
                if (selectedOpt == 1)
                {
                  arch_angle += 5.0f;
                  arch_angle = constrain(arch_angle, 0.0f, arch_angle);
                }
                if (selectedOpt == 2)
                {
                  arch_time += 100.0f;
                  arch_time = constrain(arch_time, 0.0f, 10000.0f);
                }
              }
            }
            else if (!repeat)
            {
              selectedOpt = 0;
              menu++;
              if (menu > menu_count - 1)
                menu = 0;
            }
          }

          else if (strcmp(m.name, "LEFT") == 0)
          {
            if (selected)
            {
              if (menu == 1)
              {
                targetYaw -= 5;
                if (targetYaw < -180)
                  targetYaw = 180;
              }
              if (menu == 2)
              {
                if (selectedOpt == 0)
                {
                  speed -= speedStep;
                  speed = constrain(speed, 0, 100);
                }
                if (selectedOpt == 1)
                {
                  max_speed -= speedStep;
                  max_speed = constrain(max_speed, 0, 100);
                }
                if (selectedOpt == 2)
                {
                  rot_speed -= speedStep;
                  rot_speed = constrain(rot_speed, 0, 100);
                }
                if (selectedOpt == 3)
                {
                  ramp_up_step -= 0.05;
                  ramp_up_step = constrain(ramp_up_step, 0, 100);
                }
              }
              if (menu == 3)
              {
                if (selectedOpt == 0)
                {
                  Kp -= stepKp;
                }
                if (selectedOpt == 1)
                {
                  Kd -= stepKd;
                }
              }
              if (menu == 4)
              {
                if (selectedOpt == 0)
                {
                  yawKp -= yawStepKp;
                }
                if (selectedOpt == 1)
                {
                  yawKd -= yawStepKd;
                }
              }
              if (menu == 5)
              {
                if (selectedOpt == 0)
                {
                  flag_threshold -= 10000;
                }
                if (selectedOpt == 1)
                {
                  threshold -= 10.0f;
                  threshold = constrain(threshold, 0.0f, 1000.0f);
                }
              }
              if (menu == 6)
              {
                if (selectedOpt == 0)
                {
                  arch_speed -= 5.0f;
                  arch_speed = constrain(arch_speed, 0.0f, 100.0f);
                }
                if (selectedOpt == 1)
                {
                  arch_angle -= 5.0f;
                  arch_angle = constrain(arch_angle, 0.0f, arch_angle);
                }
                if (selectedOpt == 2)
                {
                  arch_time -= 100.0f;
                  arch_time = constrain(arch_time, 0.0f, 10000.0f);
                }
              }
            }
            else if (!repeat)
            {
              selectedOpt = 0;
              menu--;
              if (menu < 0)
                menu = menu_count - 1;
            }
          }
          else if (strcmp(m.name, "UP") == 0 && !repeat)
          {
            if (!selected)
            {
              selectedOpt--;
              if (selectedOpt < 0)
                selectedOpt = menus[menu].optCount - 1;
            }
          }
          else if (strcmp(m.name, "DOWN") == 0 && !repeat)
          {
            if (!selected)
            {
              selectedOpt++;
              if (selectedOpt > menus[menu].optCount - 1)
                selectedOpt = 0;
            }
          }
          else if (strcmp(m.name, "1") == 0 && !repeat)
          {
            if (selected && menu == 1)
              targetYaw = -180;
            if (menu == 0)
              mode = 1;
          }
          else if (strcmp(m.name, "2") == 0 && !repeat)
          {
            if (selected && menu == 1)
              targetYaw = -90;
            if (menu == 0)
              mode = 2;
          }
          else if (strcmp(m.name, "3") == 0 && !repeat)
          {
            if (selected && menu == 1)
              targetYaw = 0;
            if (menu == 0)
              mode = 3;
          }
          else if (strcmp(m.name, "4") == 0 && !repeat)
          {
            if (selected && menu == 1)
              targetYaw = 90;
            if (menu == 0)
              mode = 4;
          }
          else if (strcmp(m.name, "5") == 0 && !repeat)
          {
            if (selected && menu == 1)
              targetYaw = 180;
            if (menu == 0)
              mode = 5;
          }
        }
        saveParams();
        break;
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