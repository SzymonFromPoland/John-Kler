#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <Adafruit_MCP23X08.h>

// Constants
#define SENSOR_COUNT 9
#define SDA 5
#define SCL 6
#define THRESHOLD 600

// Sensor addresses
const uint8_t sensorAddresses[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
const int xshuts[SENSOR_COUNT] = {-1, 5, 6, 7, 4, 0, 1, 2, 3};

// Sensor objects
VL53L4CD sensors[SENSOR_COUNT] = {
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1),
    VL53L4CD(&Wire, -1)
};

Adafruit_MCP23X08 mcp;

void setup()
{
    Serial.begin(115200);
    delay(2500);
    Serial.println("\n\n=== VL53L4CD Sensor Test ===");

    // Initialize I2C
    Wire.begin(SDA, SCL);
    Wire.setClock(400000);
    Serial.println("I2C initialized at 400kHz");

    // Initialize MCP23008
    Serial.println("Initializing MCP23008...");
    mcp.begin_I2C(0x20, &Wire);
    
    for (int i = 0; i < 8; i++)
    {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }
    Serial.println("MCP23008 initialized");

    // Initialize sensors
    Serial.println("Initializing sensors...");
    
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        delay(50);
        if (xshuts[i] >= 0)
            mcp.digitalWrite(xshuts[i], HIGH);
        delay(50);
        
        sensors[i].InitSensor();
        sensors[i].VL53L4CD_SetI2CAddress(sensorAddresses[i]);
        
        Serial.printf("Sensor %d initialized at address 0x%02X\n", i, sensorAddresses[i]);
    }

    // Set timing budget and start ranging
    Serial.println("Starting ranging...");
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].VL53L4CD_SetRangeTiming(10, 0);
        sensors[i].VL53L4CD_StartRanging();
    }
    
    Serial.println("Setup complete\n");
}

void loop()
{
    unsigned long readStart = millis();
    VL53L4CD_Result_t results;
    uint8_t NewDataReady = 0;
    uint16_t distances[SENSOR_COUNT];

    // Wait for first sensor to have data ready
    do
    {
        sensors[0].VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (NewDataReady == 0);

    // Read all sensors
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].VL53L4CD_ClearInterrupt();
        sensors[i].VL53L4CD_GetResult(&results);
        distances[i] = (results.range_status == 0) ? results.distance_mm : THRESHOLD;
    }

    unsigned long readTime = millis() - readStart;

    // Print timing and all 9 sensor distances
    Serial.printf("%dms", readTime);
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.printf("\t%d", distances[i]);
    }
    Serial.println();
}
