#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <Adafruit_MCP23X08.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

#include <config.h>
#include <motors.h>
#include <sensors.h>
#include <start_module.h>
#include <dashboard.h>

Preferences prefs_global;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MPU6050 mpu;
Servo servo;

bool callibrate_mpu = true;

int mode = 1;
float error = 0.0f;

float Kp = 800.0f;
float Kd = 65.0f;

float speed = 40.0f;
float max_speed = 40.0f;
int last_dir = -1;

uint16_t dist[SENSOR_COUNT];

unsigned long slowUntil = 0;
float slowSpeed = 20;
const int SLOW_THRESHOLD = threshold;

bool doCalibrate = false;

float yaw = 0.0f;
float targetYaw = 0.0f;
float lastTargetYaw = 0.0f;

float gxBias = 0;

void calibrateGyro()
{
    if (mpu.testConnection() && callibrate_mpu)
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

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        int samples = 200;
        long sum = 0;
        int16_t ax, ay, az, gx, gy, gz;

        for (int i = 0; i < samples; i++)
        {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            sum += gx;
            delay(5);
        }
        gxBias = (float)sum / samples / 131.0f;

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
    // servo.attach(SERVO, 500, 2500);
    startDashboard(dist, &mode, &Kp, &Kd, &speed, &max_speed, &targetYaw, &doCalibrate);

    mpu.initialize();
}

float prev_error = 0.0f;
float pd(float error, float dt, float Kp, float Kd)
{
    static float prev_error = 0.0f;
    static float prev_derivative = 0.0f;

    if (dt < 0.001f)
        dt = 0.001f;
    float P = Kp * error;
    float derivative = (error - prev_error) / dt;
    derivative = prev_derivative * derivative;

    prev_derivative = derivative;
    prev_error = error;

    float D = Kd * derivative;

    float output = P + D;

    return output;
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
unsigned long lastDisplayUpdate = 0;
unsigned long targetTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
const unsigned long TARGET_ARROW_SHOW_TIME = 1000;

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

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float rate = ((float)gx / 131.0f) - gxBias;
    yaw += rate * dt;

    float diff = targetYaw - yaw;
    while (diff > 180)
        diff -= 360;
    while (diff < -180)
        diff += 360;

    float yawOutput = pd(diff, dt, 1.5f, 0.67f);

    float currentMax = speed;
    if (abs(diff) < 30.0f)
    {
        currentMax = map(abs(diff), 0, 30, 15, speed);
    }

    if (abs(diff) < 3.0f)
    {
       drive(0, 0);
       yawOutput=0;
    }

    if (started)
    {
        float leftSpeed = constrain(yawOutput, -currentMax, currentMax);
        float rightSpeed = constrain(-yawOutput, -currentMax, currentMax);
        drive(leftSpeed, rightSpeed);
    }
    else
    {
        drive(0, 0);
    }

    // read_sensors(dist);
    error = calc_error(dist);

    if (error < -0.01)
        last_dir = -1;
    else if (error > 0.1)
        last_dir = 1;

    //   bool none = true;
    //   bool anyUnderThreshold = false;
    //   for (int i = 0; i < SENSOR_COUNT; i++)
    //   {
    //     if (dist_ut[i])
    //       none = false;
    //     if (dist[i] < SLOW_THRESHOLD)
    //       anyUnderThreshold = true;
    //   }

    float output = pd(error, dt, Kp / 100, Kd / 100);

    //   int left = 0, right = 0;
    //   if (started)
    //   {
    //     if (none)
    //     {
    //       float rotSpeed = speed;
    //       left = last_dir * rotSpeed;
    //       right = -last_dir * rotSpeed;
    //     }
    //     else
    //     {
    //       left = speed + output;
    //       right = speed - output;
    //     }

    //     if (dist[4] > 100)
    //       start_wait = millis();

    //     if (millis() - start_wait > 850)
    //       drive(100, 100);
    //     else
    //       drive(constrain(left, -max_speed, max_speed), constrain(right, -max_speed, max_speed));
    //   }
    //   else
    //   {
    //     drive(0, 0);
    //   }

    if (targetYaw != lastTargetYaw)
        targetTime = millis();

    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL && !started)
    {
        lastDisplayUpdate = millis();

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setDrawColor(1);

        // targetYaw += 4;

        // if (targetYaw > 180)
        //     targetYaw = -180;

        // if (millis() - targetTime < TARGET_ARROW_SHOW_TIME)
        if (true)
        {
            u8g2.setCursor(0, 10);
            u8g2.print("TY: ");
            u8g2.print(targetYaw, 1);
            u8g2.setCursor(0, 20);
            u8g2.print("CY: ");
            u8g2.print(diff, 1);
            u8g2.setCursor(0, 30);
            u8g2.print("YO: ");
            u8g2.print(yawOutput, 1);
            u8g2.drawCircle(80, 16, 15);
            u8g2.drawLine(80, 16, 80 + (int)(14 * sin((targetYaw - yaw) * DEG_TO_RAD)), 16 - (int)(14 * cos((targetYaw - yaw) * DEG_TO_RAD)));
        }
        else
        {

            u8g2.setCursor(0, 10);
            u8g2.print(started ? "ON " : "OFF");
            u8g2.print(" M");
            u8g2.print(mode);
            u8g2.print(" ");
            u8g2.print(readTime);
            u8g2.print("ms ");
            u8g2.print(yaw);
            u8g2.print("deg ");

            u8g2.setCursor(0, 20);
            u8g2.print("e:");
            u8g2.print(error, 2);
            u8g2.print(" o:");
            u8g2.print(output, 2);

            for (int i = 0; i < SENSOR_COUNT; i++)
            {
                int x = i * 8;
                if (dist[i] < threshold)
                    u8g2.drawBox(x, 24, 7, 8);
                else
                    u8g2.drawFrame(x, 24, 7, 8);
            }
        }

        u8g2.sendBuffer();
    }

    lastTargetYaw = targetYaw;

    readTime = millis() - readStart;
    Serial.printf("%lums\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\n", readTime, dist[0], dist[1], dist[2], dist[3], dist[4], dist[5], dist[6], dist[7], dist[8], targetYaw);
}