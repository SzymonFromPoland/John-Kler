#include <Arduino.h>
#include <Wire.h>
#include <VL53L4CD.h>
#include <Adafruit_MCP23X08.h>
#include <Preferences.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

#include <config.h>
#include <motors.h>
#include <sensors.h>
#include <start_module.h>
#include "dashboard.h"

Preferences prefs_global;
Adafruit_MCP23X08 mcp;
VL53L4CD sensors[SENSOR_COUNT];
Adafruit_SSD1306 display(128, 32, &Wire, -1);
Servo servo;

int mode = 1;
float error = 0.0f;

float Kp = 50.0f;
float Kd = 20.0f;
float speed = 40.0f;
int last_dir = -1;

uint16_t dist[SENSOR_COUNT];
bool dist_ut[SENSOR_COUNT];

void setup()
{
  Serial.begin(115200);
  delay(2500);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  esp_task_wdt_deinit();
  Wire.begin(SDA, SCL);
  Wire.setClock(1000000);

  setup_mcp();
  setup_sensors();
  setup_motors();
  startIRTask(RCV);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  drive(0, 0);
  servo.attach(SERVO, 500, 2500);
  startDashboard(dist, &mode, &Kp, &Kd, &speed);
}

float prev_error = 0;
float pd(float error, float dt, float Kp, float Kd)
{
  if (dt < 0.001f) dt = 0.001f;
  float proportional = Kp * error;
  float deritative = Kd * (error - prev_error) / dt;
  prev_error = error;
  return proportional + deritative;
}

float dt;
unsigned long lastTime = 0;
void loop()
{
  unsigned long loopStart = millis();
  if (lastTime == 0) lastTime = loopStart;
  dt = (loopStart - lastTime) / 1000.0f;
  lastTime = loopStart;

  read_sensors(dist, dist_ut, &error);

  if (error < -0.01)
    last_dir = -1;
  else if (error > 0.1)
    last_dir = 1;

  bool none = true;
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (dist_ut[i])
    {
      none = false;
      break;
    }

  float output = pd(error, dt, Kp, Kd);

  int left = 0, right = 0;

  if (started)
  {
    if (none)
    {
      left = last_dir * speed;
      right = -last_dir * speed;
    }
    else
    {
      left = speed + output;
      right = speed - output;
    }
    drive(left, right);
  }
  else
  {
    drive(0, 0);
    prev_error = 0;
    lastTime = 0;
  }

  unsigned long loopTime = millis() - loopStart;
  Serial.printf("L:%d R:%d err:%.2f out:%.2f\n", left, right, error, output);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(started ? "RUN" : "STP");
  display.print(" M");
  display.print(mode);
  display.print(" ");
  display.print(loopTime);
  display.print("ms");

  display.setCursor(0, 8);
  display.print("e:");
  display.print(error, 2);
  display.print(" o:");
  display.print(output, 1);

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    int x = i * 8;
    if (dist_ut[i])
      display.fillRect(x, 24, 7, 8, SSD1306_WHITE);
    else
      display.drawRect(x, 24, 7, 8, SSD1306_WHITE);
  }

  display.display();
}
