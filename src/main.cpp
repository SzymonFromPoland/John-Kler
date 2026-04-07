#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

#include <config.h>
#include <motors.h>
#include <sensors.h>
#include <controller.h>
#include <intro.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs_global;
Adafruit_MPU6050 mpu;
Servo servo;

VL53L4CD_Result_t results[SENSOR_COUNT];
uint16_t dist[SENSOR_COUNT];
bool flag[SENSOR_COUNT];

int last_dir = -1;
bool reached_yaw = false;
bool close_to_yaw = false;
bool servo_toggle = true;

float yaw = 0.0f;
float lastTargetYaw = 0.0f;
float gxBias = 0;

void calibrateGyro()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_spleen8x16_me);

    const char *line1 = "CALIBRATING...";
    const char *line2 = "DON'T MOVE ROBOT";

    u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth(line1)) / 2, 16);
    u8g2.println(line1);

    u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth(line2)) / 2, 32);
    u8g2.println(line2);

    u8g2.sendBuffer();
    delay(1000);

    int samples = 200;
    float sum = 0;
    sensors_event_t a, g, temp;

    for (int i = 0; i < samples; i++)
    {
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.x;
    }
    gxBias = sum / (float)samples;

    u8g2.clearBuffer();
    const char *done1 = "CALIBRATION";
    const char *done2 = "DONE";

    u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth(done1)) / 2, 16);
    u8g2.println(done1);

    u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth(done2)) / 2, 32);
    u8g2.println(done2);

    u8g2.sendBuffer();
    delay(1000);
}

float estimateTime(float distance_m, float speed_percent, float k = 1.0f)
{
    const float wheel_diameter = 0.034f;
    const float rpm_max = 1200.0f;

    float v = (speed_percent / 100.0f) * (rpm_max / 60.0f) * PI * wheel_diameter * k;

    if (v < 0.01f)
        return 9999.0f;

    return distance_m / v;
}

void setup()
{
    Serial.begin(115200);

    esp_task_wdt_deinit();

    Wire.begin(SDA, SCL);
    Wire.setClock(400000);

    setup_sensors();
    setup_motors();
    startIRTask(RCV);

    u8g2.begin();

    drive(0, 0);

    gpio_reset_pin((gpio_num_t)SERVO);
    pinMode(SERVO, OUTPUT);
    pinMode(BAT, INPUT);
    servo.attach(SERVO);

    if (mpu.begin(0x68))
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
        mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        mpu.setSampleRateDivisor(0);
        mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    }
    else
    {
        u8g2.clearBuffer();
        const char *done1 = "MPU6050";
        const char *done2 = "NOT FOUND";

        u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth(done1)) / 2, 16);
        u8g2.println(done1);

        u8g2.setCursor((u8g2.getDisplayWidth() - u8g2.getStrWidth(done2)) / 2, 32);
        u8g2.println(done2);

        u8g2.sendBuffer();
        delay(750);
    }

    if (play_intro)
    {
        for (int i = 0; i < 5; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                u8g2.clearBuffer();

                u8g2.drawBitmap(
                    0, 0,
                    16, 31,
                    epd_bitmap_allArray[j]);

                u8g2.sendBuffer();
                delay(100);
            }
        }
    }
}

struct PDState
{
    float prev_error = 0;
    float prev_derivative = 0;
    float integral = 0;
};

PDState linePD, yawPD;

float pid(float error, float dt, float Kp, float Ki, float Kd, PDState &state, float alpha = 1.0f, float integral_limit = 1000.0f)
{
    if (dt < 0.002f)
        dt = 0.002f;

    float P = Kp * error;

    state.integral += error * dt;
    state.integral = constrain(state.integral, -integral_limit, integral_limit);
    float I = Ki * state.integral;
    float raw_derivative = (error - state.prev_error) / dt;
    float derivative = alpha * raw_derivative + (1.0f - alpha) * state.prev_derivative;

    state.prev_error = error;
    state.prev_derivative = derivative;

    return P + I + Kd * derivative;
}

float calc_error(uint16_t *distances)
{
    float eps = 1e-3f;
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float s = 1.0f / (distances[i] + eps);
        int position = i - 4;
        numerator += s * position;
        denominator += s;
    }

    return (denominator > 0.0001f) ? (numerator / denominator) : 0.0f;
}

float dt = 0;
unsigned long lastTime = 0;
unsigned long readTime = 0;
unsigned long targetTime = 0;
unsigned long reachedYawTime = 0;
unsigned long servoTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long archTime = 0;
unsigned long attackTime = 0;
unsigned long slowDownTime = 0;

const unsigned long SERVO_ROTATION_TIME = 180;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
const unsigned long TARGET_ARROW_SHOW_TIME = 1000;
const unsigned long SLOW_DOWN_TIME = 1000;
unsigned long ATTACK_MISSED_TIME = 0;

// TODO - add tactics
// [x] -    M1 tornado
// [x] -    M2 kat i pizda (czujniki < 100)
// [x] -    M3 kat, pizda i (czujniki < threshold)
// [x] -    M4 łuk flagowys
// [x] -    M5 powolny podjazd
// [ ] -    M6 flaga i czeka
// [x] - spowalnianie na blisko
// [ ] - tune drive pid
// [x] - wykrywanie flagi

void handleServo()
{

    if (!started)
    {
        servo_toggle = true;
        move_servo = false;
    }

    if (test_servo)
    {
        servo.write(0);
        if (millis() - servoTime > SERVO_ROTATION_TIME)
        {
            test_servo = false;
        }
    }
    else if (servo_toggle && move_servo)
    {
        servo.write(0);
        if (millis() - servoTime > SERVO_ROTATION_TIME)
        {
            servo_toggle = false;
        }
    }
    else
    {
        servo.write(servo_midpoint);
        servoTime = millis();
    }
}

float yaw_output = 0.0f;
int arch_dir = -1;
float leftSpeed;
float rightSpeed;
float ramp_up1;

void loop()
{
    unsigned long readStart = millis();
    dt = (readStart - lastTime) / 1000.0f;
    lastTime = readStart;

    handleIR();

    if (doCalibrate && !started)
    {
        drive(0, 0);
        calibrateGyro();
        yaw = 0;
        doCalibrate = false;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float rate = (g.gyro.x - gxBias) * RAD_TO_DEG;
    yaw += rate * dt;

    yaw = fmod(yaw + 360.0f, 360.0f);
    float yaw_diff = targetYaw - yaw;
    yaw_diff = fmod(yaw_diff + 540.0f, 360.0f) - 180.0f;

    yaw_output = pid(yaw_diff, dt, yawKp, 0.0f, yawKd, yawPD, 0.75f);

    if (abs(yaw_diff) <= 5.0f && !reached_yaw)
        reachedYawTime = reachedYawTime;
    else
        reachedYawTime = millis();

    if (millis() - reachedYawTime >= 90)
        reached_yaw = true;

    close_to_yaw = abs(yaw_diff) <= 25.0f;

    if (mode == 1 || dyn_mode == 1)
    {
        read_sensors(results);
    }
    else if (mode == 2 || dyn_mode == 2)
    {
        if (reached_yaw || !started)
            read_sensors(results);
    }
    else if (mode == 3 || dyn_mode == 3)
    {
        if (close_to_yaw || !started)
            read_sensors(results);
    }
    else if (mode == 4 || dyn_mode == 4)
    {
        if (close_to_yaw || !started)
            read_sensors(results);
    }
    else if (mode == 5 || dyn_mode == 5)
    {
        if (reached_yaw || !started)
            read_sensors(results);
    }
    else if (mode == 6 || dyn_mode == 6)
    {
        if (reached_yaw || !started)
            read_sensors(results);
    }

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        dist[i] = results[i].distance_mm;
        results[i].number_of_spad = (results[i].number_of_spad > 0) ? results[i].number_of_spad : 1;
        float index = ((float)results[i].signal_per_spad_kcps * ((float)dist[i] * (float)dist[i])) / (float)results[i].number_of_spad;
        flag[i] = (index > flag_threshold);
        dist[i] = (detect_flag && flag[i]) ? threshold : dist[i];
    }

    float error = calc_error(dist);
    float alpha = 1.0f;
    if (mode == 5)
        alpha = 0.5f;
    float output = pid(error, dt, Kp, 0.0f, Kd, linePD, alpha);

    if (error > 0.1)
        last_dir = 1;
    else if (error < -0.1)
        last_dir = -1;

    bool any_ut1 = false;
    bool any_ut2 = false;
    bool any_ut3 = false;
    bool speed_up = false;
    for (uint16_t d : dist)
    {
        if (d < threshold)
        {
            any_ut1 = true;
        }
        if (d < 100)
        {
            any_ut2 = true;
        }
        if (slow_down && d < slow_threshold)
        {
            any_ut3 = true;
        }
    }

    if (!any_ut3)
        slowDownTime = millis();

    speed_up = millis() - slowDownTime > SLOW_DOWN_TIME;

    if (!reached_yaw)
        archTime = millis();

    if (!close_to_yaw)
        attackTime = millis();

    if (started)
    {
        move_servo = true;
        if (mode == 1 || dyn_mode == 1)
        {
            if (any_ut1)
            {

                if (dist[3] < threshold || dist[4] < threshold || dist[5] < threshold)
                    ramp_up1 = constrain(ramp_up1 + ramp_up_step, -max_speed, max_speed);
                if (slow_down && any_ut3 && !speed_up)
                    ramp_up1 = 0.0f;
                leftSpeed = ((slow_down && any_ut3 && !speed_up) ? slow_speed : (speed + ramp_up1)) + output;
                rightSpeed = ((slow_down && any_ut3 && !speed_up) ? slow_speed : (speed + ramp_up1)) - output;
            }
            else
            {
                ramp_up1 = 0.0f;
                leftSpeed = speed * last_dir;
                rightSpeed = speed * -last_dir;
            }
        }
        else if (mode == 2 || dyn_mode == 2)
        {
            if (any_ut2 || millis() - attackTime > ATTACK_MISSED_TIME)
                dyn_mode = 1;
            leftSpeed = (close_to_yaw ? rot_speed : 0) + yaw_output;
            rightSpeed = (close_to_yaw ? rot_speed : 0) - yaw_output;
        }
        else if (mode == 3 || dyn_mode == 3)
        {
            if (any_ut1 || millis() - attackTime > ATTACK_MISSED_TIME)
                dyn_mode = 1;
            leftSpeed = (close_to_yaw ? rot_speed : 0) + yaw_output;
            rightSpeed = (close_to_yaw ? rot_speed : 0) - yaw_output;
        }
        else if (mode == 4 || dyn_mode == 4)
        {
            if (millis() - archTime < arch_time && reached_yaw)
            {
                leftSpeed = arch_speed + (arch_dir * arch_angle);
                rightSpeed = arch_speed - (arch_dir * arch_angle);
            }
            else
            {
                leftSpeed = yaw_output;
                rightSpeed = -yaw_output;

                if (reached_yaw)
                    dyn_mode = 1;
            }
        }
        else if (mode == 5 || dyn_mode == 5)
        {
            leftSpeed = yaw_output;
            rightSpeed = -yaw_output;

            if (reached_yaw)
            {
                leftSpeed = 15 + output;
                rightSpeed = 15 - output;

                if (any_ut2)
                    dyn_mode = 1;
            }
        }
    }
    else
    {
        ramp_up1 = 0.0f;
        dyn_mode = mode;
        arch_dir = (yaw_diff > 0) ? 1 : -1;
        reached_yaw = false;
        ATTACK_MISSED_TIME = estimateTime(0.325f, rot_speed) * 1000.0f;
    }

    leftSpeed = constrain(leftSpeed, -max_speed, max_speed);
    rightSpeed = constrain(rightSpeed, -max_speed, max_speed);

    drive(started ? leftSpeed : 0, started ? rightSpeed : 0);

    handleServo();

    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL)
    {
        lastDisplayUpdate = millis();

        static bool lastFlipped = false;
        if (screen_flipped != lastFlipped)
        {
            if (screen_flipped)
                u8g2.setDisplayRotation(U8G2_R2);
            else
                u8g2.setDisplayRotation(U8G2_R0);

            lastFlipped = screen_flipped;
        }

        Menu &m = menus[menu];
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setDrawColor(1);

        // Adaptive Helper: Handles paging, highlighting, and labels automatically
        auto drawOption = [&](int index, const char *label, float value, int precision, bool isBool = false)
        {
            int page = selectedOpt / 2;
            int itemPage = index / 2;
            if (page != itemPage)
                return;

            int y = 20 + (index % 2) * 10;

            if (selectedOpt == index)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y - 8, 128, 10);
                    u8g2.setDrawColor(0);
                }
                u8g2.setCursor(0, y);
                u8g2.print(">");
                u8g2.setDrawColor(selected ? 0 : 1);
            }

            u8g2.setCursor(8, y);
            u8g2.print(label);
            if (isBool)
                u8g2.print(value > 0 ? "true" : "false");
            else
                u8g2.print(value, precision);
            u8g2.setDrawColor(1);
        };

        switch (m.id)
        {
        case 0:
            u8g2.setCursor(0, 10);
            u8g2.print(started ? "ON " : "OFF");
            u8g2.print(" M");
            u8g2.print(mode);
            u8g2.print(" FD: ");
            u8g2.print(detect_flag ? "YES " : "NO  ");
            u8g2.print(readTime);
            u8g2.print("ms");

            u8g2.setCursor(0, 20);
            u8g2.print("e: ");
            u8g2.print(error, 2);
            u8g2.print(" o: ");
            u8g2.print(output, 2);

            for (int i = 0; i < SENSOR_COUNT; i++)
            {
                int x = i * 8;
                int idx = screen_flipped ? SENSOR_COUNT - (i + 1) : i;
                if (detect_flag && flag[idx])
                {
                    u8g2.drawFrame(x, 24, 7, 8);
                    u8g2.drawBox(x + 2, 26, 3, 4);
                }
                else if (dist[idx] < threshold)
                    u8g2.drawBox(x, 24, 7, 8);
                else
                    u8g2.drawFrame(x, 24, 7, 8);
            }
            u8g2.setCursor(75, 32);
            u8g2.print("B: ");
            u8g2.print(analogRead(BAT) * 0.00349f, 1);
            u8g2.print("V");
            break;

        case 1:
            u8g2.setCursor(1, 10);
            if (selected)
            {
                u8g2.drawBox(0, 2, 60, 10);
                u8g2.setDrawColor(0);
            }
            u8g2.print("TY: ");
            u8g2.print(targetYaw, 1);
            u8g2.setDrawColor(1);
            u8g2.setCursor(1, 20);
            u8g2.print("DY: ");
            u8g2.print(yaw_diff, 1);
            u8g2.setCursor(1, 30);
            u8g2.print("YO: ");
            u8g2.print(yaw_output, 1);
            u8g2.drawCircle(80, 16, 15);
            u8g2.drawLine(80, 16, 80 + (int)(14 * sin((180 * screen_flipped + yaw_diff) * DEG_TO_RAD)), 16 - (int)(14 * cos((180 * screen_flipped + yaw_diff) * DEG_TO_RAD)));
            break;

        case 2:
            u8g2.setCursor(0, 10);
            u8g2.print("Speed Control");
            drawOption(0, "Speed: ", speed, 0);
            drawOption(1, "MAX:   ", max_speed, 0);
            drawOption(2, "Rot:   ", rot_speed, 0);
            drawOption(3, "Ramp:  ", ramp_up_step, 2);
            drawOption(4, "Slow:  ", slow_speed, 0);
            break;

        case 3:
            u8g2.setCursor(0, 10);
            u8g2.print("Drive PD");
            drawOption(0, "Kp: ", Kp, 4);
            drawOption(1, "Kd: ", Kd, 4);
            break;

        case 4:
            u8g2.setCursor(0, 10);
            u8g2.print("Gyro PD");
            drawOption(0, "Kp: ", yawKp, 6);
            drawOption(1, "Kd: ", yawKd, 6);
            break;

        case 5:
            u8g2.setCursor(0, 10);
            u8g2.print("Thresholds");
            drawOption(0, "FDTh: ", flag_threshold, 0);
            drawOption(1, "STh:  ", threshold, 0);
            drawOption(2, "SDTh: ", slow_threshold, 0);
            break;

        case 6:
            u8g2.setCursor(0, 10);
            u8g2.print("Arch Control");
            drawOption(0, "Speed: ", arch_speed, 1);
            drawOption(1, "Angle: ", arch_angle, 1);
            drawOption(2, "Time:  ", arch_time, 0);
            break;

        case 7:
            u8g2.setCursor(0, 10);
            u8g2.print("Extra Settings");
            drawOption(0, "Slow:  ", slow_down, 0, true);
            drawOption(1, "Intro: ", play_intro, 0, true);
            break;
        }
        u8g2.sendBuffer();
    }
    readTime = millis() - readStart;
}