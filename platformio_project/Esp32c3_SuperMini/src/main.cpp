#include <Arduino.h>
#include "WiFi.h"
#include "Adafruit_VL53L0X.h"

#include "wifi_config.h"

#define PIN_LED_BUILTIN 8 // ACTUALLY ITS 8 INSTAED OF 7 THAT IS DEFINED IN LED_BUILTIN

#define PIN_SDA_PIN 8
#define PIN_SCL_PIN 9
#define PIN_BUZZER 10

const double RANGE_MODIFIER = 0.006;
const double RANGE_EXPONENT = 3.0;
const int MEASUREMENTS_LEN = 3;

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

void playMelody()
{
  int melody[] = {523, 494, 587, 659, 440, 494, 349};
  int duration[] = {150, 100, 200, 150, 300, 100, 350};

  for (int i = 0; i < 7; i++) {
    tone(PIN_BUZZER, melody[i], duration[i]);
    delay(duration[i] + 50);
  }
}

void setup()
{
  Serial.begin(9600); // whatever, usb speed anyway
  pinMode(PIN_BUZZER, OUTPUT);
  Wire.begin(8, 9);

  playMelody();

  // sensor 1 begin

  while(!sensor.begin(0x30))
  {
    tone(PIN_BUZZER, 2000, 200);
    delay(200);
    tone(PIN_BUZZER, 2000, 200);
    delay(5000);
  }

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
    }
  }

  sensor.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  sensor.startRangeContinuous();

  int measurements[MEASUREMENTS_LEN] = {0};
  int measurements_delta[MEASUREMENTS_LEN] = {0};
  short measurements_position_counter = 0;
  int avarge = 10;
  int tempRange;
  unsigned long start_time_ms;
  while (true)
  {
    start_time_ms = millis();

    while (millis() - pow(avarge*RANGE_MODIFIER, RANGE_EXPONENT) <= start_time_ms)
    {
      if (sensor.isRangeComplete())
      {
        tempRange = sensor.readRange();
        tempRange = min(tempRange, 2500);
        measurements[measurements_position_counter] = tempRange;
        measurements_position_counter ++ ;
        if (measurements_position_counter == MEASUREMENTS_LEN)
        {
          int sum = 0;
          for (int i = 0; i < MEASUREMENTS_LEN; i++)
          {
            sum += measurements[i];
          }
          avarge = (float)sum / (float)MEASUREMENTS_LEN;
          int max_delta = 0;
          for (int i = 0; i < MEASUREMENTS_LEN; i++)
          {
            measurements_delta[i] = abs(measurements[i] - avarge);
            max_delta = max(measurements_delta[i], max_delta);
          }
          for (int i = 0; i < MEASUREMENTS_LEN; i++)
          {
            if (measurements_delta[i] == max_delta)
            {
              measurements[i] = -1;
              break;
            }
          }
          sum = 0;
          for (int i = 0; i < MEASUREMENTS_LEN; i++)
          {
            int val = measurements[i]!=-1?measurements[i]:0;
            sum += val;
          }
          avarge = (float)sum / ((float)MEASUREMENTS_LEN - 1.0);
          measurements_position_counter = 0;
        }
        Serial.println(avarge); // ~~~~~~~~~~~~~~~~~~~~~~
      }

    }
    tone(PIN_BUZZER, 1000, 10);
    delay(50);
  }
}
 
void loop()
{

}
