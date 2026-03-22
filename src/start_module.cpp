#include "start_module.h"

static TaskHandle_t IRTaskHandle;
bool started = false;
bool hold_led = false;

void irTask(void *parameter)
{
  uint8_t pin = ((uint32_t *)parameter)[0];
  uint8_t ledPin = ((uint32_t *)parameter)[1];

  RC5 rc5(pin);

  uint8_t START, STOP;

  prefs_global.begin("robot", true);
  STOP = prefs_global.getUInt("stop_address", 0);
  START = prefs_global.getUInt("start_address", 0);
  prefs_global.end();

  for (;;)
  {
    unsigned char toggle;
    unsigned char address;
    unsigned char command;

    if (rc5.read(&toggle, &address, &command))
    {
      if (address == 0x0B)
      {
        START = command + 1;
        STOP = command;

        prefs_global.begin("robot", false);
        prefs_global.putUInt("stop_address", STOP);
        prefs_global.putUInt("start_address", START);
        prefs_global.end();

        hold_led = true;

        if (ledPin != -1)
        {
          for (int i = 0; i < 5; i++)
          {
            digitalWrite(ledPin, HIGH);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            digitalWrite(ledPin, LOW);
            vTaskDelay(50 / portTICK_PERIOD_MS);
          }
        }
        else
        {
          vTaskDelay(250 / portTICK_PERIOD_MS);
        }

        hold_led = false;      }
      else if (address == 0x07)
      {
        if (command == START)
          started = true;
        else if (command == STOP)
          started = false;
      }
    }
  }
}

void startIRTask(uint8_t pin, uint8_t ledPin)
{
  uint32_t *params = (uint32_t *)malloc(2 * sizeof(uint32_t));
  params[0] = pin;
  params[1] = ledPin;

  xTaskCreatePinnedToCore(
      irTask,
      "IR_Task",
      8192,
      params,
      1,
      &IRTaskHandle,
      1);
}