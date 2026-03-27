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

float Kp = 800.0f;
float Kd = 65.0f;
float speed = 40.0f;
float max_speed = 40.0f;
int last_dir = -1;

uint16_t dist[SENSOR_COUNT];
bool dist_ut[SENSOR_COUNT];

unsigned long slowUntil = 0;
float slowSpeed = 20;
const int SLOW_THRESHOLD = threshold;

void setup()
{
  // Serial.begin(115200);
  // delay(2500);
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
  startDashboard(dist, &mode, &Kp, &Kd, &speed, &max_speed);
}

float prev_error = 0.0f;
float filtered_error = 0.0f;

float pd(float error, float dt, float Kp, float Kd)
{
  if (dt < 0.001f)
    dt = 0.001f;

  float proportional = Kp * error;

  const float alpha = 0.9f;
  filtered_error = alpha * error + (1 - alpha) * filtered_error;

  float derivative = Kd * ((filtered_error - prev_error) / dt);

  const float max_derivative = speed;
  if (derivative > max_derivative)
    derivative = max_derivative;
  if (derivative < -max_derivative)
    derivative = -max_derivative;

  prev_error = filtered_error;
  return proportional + derivative;
}

long lastTime = 0;
long start_wait = 0;

void loop()
{
  unsigned long loopStart = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  read_sensors(dist, dist_ut, &error);

  if (error < -0.01)
    last_dir = -1;
  else if (error > 0.1)
    last_dir = 1;

  bool none = true;
  bool anyUnderThreshold = false;
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (dist_ut[i])
      none = false;
    if (dist[i] < SLOW_THRESHOLD)
      anyUnderThreshold = true;
  }

  float output = pd(error, dt, Kp / 100, Kd / 100);

  int left = 0, right = 0;
  if (started)
  {
    if (none)
    {
      float rotSpeed = speed;
      left = last_dir * rotSpeed;
      right = -last_dir * rotSpeed;
    }
    else
    {
      left = speed + output;
      right = speed - output;
    }

    if (dist[4] > 100)
      start_wait = millis();

    if (millis() - start_wait > 850)
      drive(100, 100);
    else
      drive(constrain(left, -max_speed, max_speed), constrain(right, -max_speed, max_speed));
  }
  else
  {
    drive(0, 0);
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