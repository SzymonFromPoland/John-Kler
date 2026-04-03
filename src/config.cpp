#include "config.h"

float stepKp = 0.5f;
float stepKd = 0.05f;
float Kp = 8.0f;
float Kd = 0.65f;

float yawStepKp = 0.01f;
float yawStepKd = 0.00001f;
float yawKp = 0.8f;
float yawKd = 0.0005f;
float targetYaw = 0.0f;

float speedStep = 5.0f;
float speed = 50.0f;
float max_speed = 50.0f;

bool started = false;
bool hold_led = false;
bool doCalibrate = false;
int mode = 1;
int menu = 0;
bool selected = false;
int selectedOpt = 0;