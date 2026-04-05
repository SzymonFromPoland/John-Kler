#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X08.h>
#include <vl53l4cd_class.h>
#include <config.h>

void setup_sensors();
void read_sensors(VL53L4CD_Result_t *results, bool ignore = false);

#endif