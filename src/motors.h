#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <config.h>

void setup_motors();
void drive(int left, int right);

#endif