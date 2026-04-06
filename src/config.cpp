#include "config.h"

int menu_count = 7;

Menu menus[] = {
    {0, 0},
    {1, 1},
    {2, 4},
    {3, 2},
    {4, 2},
    {5, 2},
    {6, 3}};

float threshold = 300.0f;

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
float ramp_up_step = 0.5f;
float speed = 50.0f;
float max_speed = 50.0f;
float rot_speed = 50.0f;

bool started = false;
bool hold_led = false;
bool doCalibrate = false;
bool move_servo = false;
bool test_servo = false;

bool selected = false;
int selectedOpt = 0;
int mode = 1;
int dyn_mode = 1;
int menu = 0;

bool detect_flag = false;
float flag_threshold = 500000.0f;
float arch_angle = 50.0f;
float arch_speed = 100.0f;
float arch_time = 1000.0f;