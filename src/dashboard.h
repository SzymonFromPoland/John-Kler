#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <stdint.h>

#define DASHBOARD_SENSOR_COUNT 9

extern bool doCalibrate;

void startDashboard(uint16_t *dist, int *mode, float *kp, float *kd, float *speed, float *mspeed, float *targetYaw, bool *calibrateFlag);

#endif
