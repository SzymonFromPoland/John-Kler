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
unsigned long servoTime = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long SERVO_ROTATION_TIME = 180;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
const unsigned long TARGET_ARROW_SHOW_TIME = 1000;

float yaw_output = 0.0f;

// [ ] - add tactics
// [x] -    M1 tornado
// [x] -    M2 kat i pizda
// [ ] -    M3 kat, pizda i czujniki
// [ ] -    M4 łuk flagowys
// [ ] -    M5 flaga i czeka
// [ ] - tune drive pid
// [x] - wykrywanie flagi
// [x] - M2 w miejscu

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
    yaw_output = pid(targetYaw - yaw, dt, yawKp, 0.0f, yawKd, yawPD, 0.75f);
    reached_yaw = (started) ? (abs(targetYaw - yaw) <= 5.0f && !reached_yaw) : 0;
    close_to_yaw = abs(targetYaw - yaw) <= 25.0f;

    if (mode == 1)
    {
        read_sensors(results);
    }
    else if (mode == 2)
    {
        if (reached_yaw || !started)
            read_sensors(results);
    }

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        dist[i] = results[i].distance_mm;
        results[i].number_of_spad = (results[i].number_of_spad > 0) ? results[i].number_of_spad : 1;
        float index = ((float)results[i].signal_per_spad_kcps * ((float)dist[i] * (float)dist[i])) / (float)results[i].number_of_spad;
        // Serial.printf("%d\t", (int)index);
        flag[i] = (index > flag_threshold);
        dist[i] = (detect_flag && flag[i]) ? threshold : dist[i];
    }
    // Serial.println();

    float error = calc_error(dist);
    float output = pid(error, dt, Kp, 0.0f, Kd, linePD);

    if (error > 0.1)
        last_dir = 1;
    else if (error < -0.1)
        last_dir = -1;

    bool any_ut1 = false;
    bool any_ut2 = false;
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
    }

    if (started)
    {
        if (mode == 1 || dyn_mode == 1)
        {
            move_servo = true;
            if (any_ut1)
            {
                if (dist[3] < threshold || dist[4] < threshold || dist[5] < threshold)
                    ramp_up1 = constrain(ramp_up1 + ramp_up_step, -max_speed, max_speed);
                leftSpeed = (speed + ramp_up1) + output;
                rightSpeed = (speed + ramp_up1) - output;
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

            move_servo = true;
            leftSpeed = (close_to_yaw ? rot_speed : 0) + yaw_output;
            rightSpeed = (close_to_yaw ? rot_speed : 0) - yaw_output;
        }
        else if (mode == 3 || dyn_mode == 3)
        {
            if (any_ut2)
                dyn_mode = 1;
            move_servo = true;
            leftSpeed = (close_to_yaw ? rot_speed : 0) + yaw_output;
            rightSpeed = (close_to_yaw ? rot_speed : 0) - yaw_output;
        }
    }
    else
    {
        ramp_up1 = 0.0f;
        dyn_mode = mode;
    }

    leftSpeed = constrain(leftSpeed, -max_speed, max_speed);
    rightSpeed = constrain(rightSpeed, -max_speed, max_speed);

    drive(started ? leftSpeed : 0, started ? rightSpeed : 0);

    handleServo();

    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL)
    {
        lastDisplayUpdate = millis();

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setDrawColor(1);

        int y1 = 20;
        int y2 = 30;

        Menu &m = menus[menu];
        switch (m.id)
        {
        case 0:
        {
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
                if (detect_flag && flag[i])
                {
                    u8g2.drawFrame(x, 24, 7, 8);
                    u8g2.drawBox(x + 2, 26, 3, 4);
                }
                else if (dist[i] < threshold)
                {
                    u8g2.drawBox(x, 24, 7, 8);
                }
                else
                {
                    u8g2.drawFrame(x, 24, 7, 8);
                }
            }

            u8g2.setCursor(75, 32);
            u8g2.print("B: ");
            u8g2.print(analogRead(BAT) * 0.00349f, 1);
            u8g2.print("V");

            break;
        }
        case 1:
        {
            u8g2.setCursor(1, 10);
            if (selected)
            {
                u8g2.setDrawColor(1);
                u8g2.drawBox(0, 2, 60, 10);
                u8g2.setDrawColor(0);
            }
            u8g2.print("TY: ");
            u8g2.print(targetYaw, 1);
            u8g2.setCursor(1, 20);
            u8g2.setDrawColor(1);
            u8g2.print("CY: ");
            u8g2.print(yaw, 1);
            u8g2.setCursor(1, 30);
            u8g2.print("YO: ");
            u8g2.print(yaw_output, 1);
            u8g2.drawCircle(80, 16, 15);
            u8g2.drawLine(80, 16, 80 + (int)(14 * sin((targetYaw - yaw) * DEG_TO_RAD)), 16 - (int)(14 * cos((targetYaw - yaw) * DEG_TO_RAD)));
            break;
        }
        case 2:
        {
            u8g2.setCursor(0, 10);
            u8g2.print("Speed control");

            int yPositions[4] = {(selectedOpt < 2) ? 20 : 100,
                                 (selectedOpt < 2) ? 30 : 100,
                                 (selectedOpt < 2) ? 100 : 20,
                                 (selectedOpt < 2) ? 100 : 30};

            if (selectedOpt == 0)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, yPositions[0] - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, yPositions[0]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[0]);
                    u8g2.print("Speed: ");
                    u8g2.print(speed, 0);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, yPositions[0]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[0]);
                    u8g2.print("Speed: ");
                    u8g2.print(speed, 0);
                }
            }
            else
            {
                u8g2.setCursor(8, yPositions[0]);
                u8g2.print("Speed: ");
                u8g2.print(speed, 0);
            }

            if (selectedOpt == 1)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, yPositions[1] - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, yPositions[1]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[1]);
                    u8g2.print("MAX:   ");
                    u8g2.print(max_speed, 0);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, yPositions[1]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[1]);
                    u8g2.print("MAX:   ");
                    u8g2.print(max_speed, 0);
                }
            }
            else
            {
                u8g2.setCursor(8, yPositions[1]);
                u8g2.print("MAX:   ");
                u8g2.print(max_speed, 0);
            }

            if (selectedOpt == 2)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, yPositions[2] - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, yPositions[2]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[2]);
                    u8g2.print("Rot:   ");
                    u8g2.print(rot_speed, 0);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, yPositions[2]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[2]);
                    u8g2.print("Rot:   ");
                    u8g2.print(rot_speed, 0);
                }
            }
            else
            {
                u8g2.setCursor(8, yPositions[2]);
                u8g2.print("Rot:   ");
                u8g2.print(rot_speed, 0);
            }

            if (selectedOpt == 3)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, yPositions[3] - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, yPositions[3]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[3]);
                    u8g2.print("Ramp:  ");
                    u8g2.print(ramp_up_step, 2);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, yPositions[3]);
                    u8g2.print(">");
                    u8g2.setCursor(8, yPositions[3]);
                    u8g2.print("Ramp:  ");
                    u8g2.print(ramp_up_step, 2);
                }
            }
            else
            {
                u8g2.setCursor(8, yPositions[3]);
                u8g2.print("Ramp:  ");
                u8g2.print(ramp_up_step, 2);
            }

            break;
        }

        case 3:
        {
            u8g2.setCursor(0, 10);
            u8g2.print("Drive PD");

            if (selectedOpt == 0)
            {
                if (selected)
                {

                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y1 - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, y1);
                    u8g2.print(">");
                    u8g2.setCursor(8, y1);
                    u8g2.print("Kp: ");
                    u8g2.print(Kp);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, y1);
                    u8g2.print(">");
                    u8g2.setCursor(8, y1);
                    u8g2.print("Kp: ");
                    u8g2.print(Kp);
                }
            }
            else
            {
                u8g2.setCursor(8, y1);
                u8g2.print("Kp: ");
                u8g2.print(Kp);
            }

            if (selectedOpt == 1)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y2 - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, y2);
                    u8g2.print(">");
                    u8g2.setCursor(8, y2);
                    u8g2.print("Kd: ");
                    u8g2.print(Kd);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, y2);
                    u8g2.print(">");
                    u8g2.setCursor(8, y2);
                    u8g2.print("Kd: ");
                    u8g2.print(Kd);
                }
            }
            else
            {
                u8g2.setCursor(8, y2);
                u8g2.print("Kd: ");
                u8g2.print(Kd);
            }

            break;
        }
        case 4:
        {
            u8g2.setCursor(0, 10);
            u8g2.print("Gyro PD");

            if (selectedOpt == 0)
            {
                if (selected)
                {

                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y1 - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, y1);
                    u8g2.print(">");
                    u8g2.setCursor(8, y1);
                    u8g2.print("Kp: ");
                    u8g2.print(yawKp, 6);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, y1);
                    u8g2.print(">");
                    u8g2.setCursor(8, y1);
                    u8g2.print("Kp: ");
                    u8g2.print(yawKp, 6);
                }
            }
            else
            {
                u8g2.setCursor(8, y1);
                u8g2.print("Kp: ");
                u8g2.print(yawKp, 6);
            }

            if (selectedOpt == 1)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y2 - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, y2);
                    u8g2.print(">");
                    u8g2.setCursor(8, y2);
                    u8g2.print("Kd: ");
                    u8g2.print(yawKd, 6);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, y2);
                    u8g2.print(">");
                    u8g2.setCursor(8, y2);
                    u8g2.print("Kd: ");
                    u8g2.print(yawKd, 6);
                }
            }
            else
            {
                u8g2.setCursor(8, y2);
                u8g2.print("Kd: ");
                u8g2.print(yawKd, 6);
            }

            break;
        }
        case 5:
        {
            u8g2.setCursor(0, 10);
            u8g2.print("Thresholds");

            if (selectedOpt == 0)
            {
                if (selected)
                {

                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y1 - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, y1);
                    u8g2.print(">");
                    u8g2.setCursor(8, y1);
                    u8g2.print("FDTh: ");
                    u8g2.print(flag_threshold);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, y1);
                    u8g2.print(">");
                    u8g2.setCursor(8, y1);
                    u8g2.print("FDTh: ");
                    u8g2.print(flag_threshold);
                }
            }
            else
            {
                u8g2.setCursor(8, y1);
                u8g2.print("FDTh: ");
                u8g2.print(flag_threshold);
            }

            if (selectedOpt == 1)
            {
                if (selected)
                {
                    u8g2.setDrawColor(1);
                    u8g2.drawBox(0, y2 - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.setCursor(0, y2);
                    u8g2.print(">");
                    u8g2.setCursor(8, y2);
                    u8g2.print("STh: ");
                    u8g2.print(threshold);
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.setCursor(0, y2);
                    u8g2.print(">");
                    u8g2.setCursor(8, y2);
                    u8g2.print("STh: ");
                    u8g2.print(threshold);
                }
            }
            else
            {
                u8g2.setCursor(8, y2);
                u8g2.print("STh: ");
                u8g2.print(threshold);
            }

            break;
        }
        }

        u8g2.sendBuffer();
    }

    readTime = millis() - readStart;
}