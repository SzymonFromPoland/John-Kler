#include <Arduino.h>
#include <Wire.h>
#include <VL53L4CD.h>
#include <Adafruit_MCP23X08.h>
#include <Preferences.h>

#include <config.h>
#include <motors.h>
#include <sensors.h>
#include <start_module.h>

#include <esp_adc_cal.h>

Preferences prefs_global;
Adafruit_MCP23X08 mcp;
VL53L4CD sensors[SENSOR_COUNT];

#define A_IPROPI 6400.0f
#define R_IPROPI_OHMS 1800
#define AVERAGE_SAMPLES 16

esp_adc_cal_characteristics_t adc_chars;

void i2cScan()
{
  Serial.println("Scanning I2C bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("  Device at 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0)
    Serial.println("  No devices found.");
  else
  {
    Serial.print("  Total: ");
    Serial.print(found);
    Serial.println(" device(s).");
  }

  delay(2500);
}

void setup()
{
  Serial.begin(115200);

  delay(2500);

  Wire.begin(SDA, SCL);
  Wire.setClock(100000);

  setup_mcp();
  setup_sensors();
  setup_motors();
  startIRTask(RCV);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  while (!started)
    delay(10);

  delay(5000);
  drive(75, 75);

  while (started)
    delay(10);

  drive(0, 0);
}

uint16_t dist[SENSOR_COUNT];
bool dist_ut[SENSOR_COUNT];

void loop()
{
  read_sensors(dist, dist_ut);

  // for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  // {
  //   Serial.print(dist[i]);
  //   Serial.print(" ");
  // }

  // uint32_t sum_adc = 0;
  // for (int i = 0; i < AVERAGE_SAMPLES; i++)
  // {
  //   sum_adc += analogRead(IPA);
  //   delayMicroseconds(120);
  // }
  // uint32_t raw = sum_adc / AVERAGE_SAMPLES;

  // uint32_t voltage_mV = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

  // float I_IPROPI_mA = (voltage_mV * 1.0f) / R_IPROPI_OHMS;
  // float I_motor_A = (I_IPROPI_mA / 1000.0f) * A_IPROPI;

  // Serial.printf("Raw ADC: %4lu | V_IPROPI: %5.3f V | I_motor: %6.3f A (%6.0f mA)\n", raw, voltage_mV / 1000.0f, I_motor_A, I_motor_A * 1000.0f);
}

// ⬜ 🔳