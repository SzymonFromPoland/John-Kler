#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_MCP23X08.h>
#include <VL53L4CD.h>
#include <config.h>

extern Adafruit_MCP23X08 mcp;
extern VL53L4CD sensors[SENSOR_COUNT];

bool setup_mcp();
bool setup_sensors();
void read_sensors(uint16_t *dist, bool *dist_ut, float *error);

#endif