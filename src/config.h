#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define M1A 16
#define M2A 17

#define M1B 10
#define M2B 9

#define SLP 13
#define OFF 15

#define NFA 18
#define NFB 11

#define IPA 4
#define IPB 8

#define SENSOR_COUNT 9

#define SCL 6
#define SDA 5

#define RCV 12

#define SERVO 1

#define BAT 7

const int servo_midpoint = 95;

struct Menu
{
    int id;
    int optCount;
};

extern Menu menus[];

extern float threshold;

extern float Kp;
extern float Kd;
extern float stepKp;
extern float stepKd;
extern float yawKp;
extern float yawKd;
extern float yawStepKp;
extern float yawStepKd;
extern float targetYaw;

extern float speedStep;
extern float speed;
extern float max_speed;
extern float ramp_up_step;
extern float rot_speed;

extern bool started;
extern bool doCalibrate;
extern bool hold_led;
extern bool move_servo;
extern bool test_servo;
extern bool detect_flag;

extern int mode;
extern int dyn_mode;
extern int menu;
extern int menu_count;
extern bool selected;
extern int selectedOpt;

extern float flag_threshold;
extern float arch_angle;
extern float arch_speed;
extern float arch_time;

#endif