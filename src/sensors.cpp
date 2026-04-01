#include <sensors.h>

const int xshuts[SENSOR_COUNT] = {-1, 5, 6, 7, 4, 0, 1, 2, 3};

VL53L4CD sensors[SENSOR_COUNT] = {
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1)};

Adafruit_MCP23X08 mcp;

void setup_sensors()
{

    Serial.println("Setting up mcp...");
    mcp.begin_I2C(0x20, &Wire);

    for (int i = 0; i < 8; i++)
    {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }

    Serial.println("Setting up sensors...");

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        delay(50);
        if (xshuts[i] >= 0)
            mcp.digitalWrite(xshuts[i], HIGH);
        delay(50);
        sensors[i].InitSensor();
        sensors[i].VL53L4CD_SetI2CAddress(0x2A + i * 2);
        Serial.printf("Sensor %d initialized at address 0x%02X\n", i, 0x2A + i * 2);
    }
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].VL53L4CD_SetRangeTiming(10, 0);
        sensors[i].VL53L4CD_StartRanging();
    }

    Serial.println("Sensors setup complete.");
}

void read_sensors(uint16_t *distances)
{
    VL53L4CD_Result_t results;
    uint8_t NewDataReady = 0;

    do
    {
        sensors[0].VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (NewDataReady == 0);

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].VL53L4CD_ClearInterrupt();
        sensors[i].VL53L4CD_GetResult(&results);
        distances[i] = (results.range_status == 0) ? results.distance_mm : threshold;
    }
}