#include <sensors.h>

bool setup_mcp()
{
    for (int i = 0; i < 10; i++)
    {
        Wire.beginTransmission(0x20);
        Wire.endTransmission();
        delay(10);
    }

    if (!mcp.begin_I2C(0x20))
        return 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }

    return 1;
}

bool setup_sensors()
{
    for (int i = 0; i < 10; i++)
    {
        Wire.beginTransmission(0x29);
        Wire.endTransmission();
        delay(10);
    }

    delay(100);

    bool status[SENSOR_COUNT];

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        delay(10);
        if (xshuts[i] != -1)
            mcp.digitalWrite(xshuts[i], HIGH);
        delay(10);

        sensors[i].setTimeout(300);
        if (!sensors[i].init(false, true))
        {
            Serial.print("Failed to init sensor ");
            Serial.println(i);
            mcp.digitalWrite(xshuts[i], LOW);
            delay(200);
            status[i] = 0;
        }
        else
        {
            sensors[i].setAddress(0x2A + i);
            sensors[i].setRangeTiming(25, 0);
            sensors[i].startContinuous();
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(" initialized at 0x");
            Serial.println(0x2A + i, HEX);
            status[i] = 1;
        }
    }

    return status;
}

void read_sensors(uint16_t *dist, bool *dist_ut, float *error)
{
    float eps = 1e-3f;

    float numerator = 0.0f;
    float denominator = 0.0f;
    uint8_t status[SENSOR_COUNT];
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].read(false);
        dist[i] = sensors[i].ranging_data.range_mm;
        status[i] = sensors[i].ranging_data.range_status;
        dist[i] = (dist[i] < threshold && status[i] == 0) ? dist[i] : threshold;
        if (sensors[i].timeoutOccurred())
            dist[i] = -1;

        dist_ut[i] = dist[i] < threshold && dist[i] != -1;

        if (!dist_ut[i]) continue;
        float s = 1.0f / (dist[i] + eps);
        int position = i - 4;
        numerator += s * position;
        denominator += s;
    }
    *error = (denominator > 0.0001f) ? (numerator / denominator) : 0.0f;
}