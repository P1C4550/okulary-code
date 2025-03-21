#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
/* FOR FUTURE USE
#include "WiFi.h"
#include "wifi_config.h"
*/
#define PIN_SDA_PIN 8 // connected HIGH side to builtin LED
#define PIN_SCL_PIN 9
#define PIN_BUZZER 10
#define PIN_XSHUT_RIGHT 2
#define PIN_XSHUT_LEFT 3

#define SENSOR_MAX_RANGE_MM 2500
#define MEASUREMENTS_COUNT 3
#define MEASUREMENTS_PROMIXITY_PERCENT 0.2 // how much of the sensors measurement is close to the other one; this defines the threshold if both sensors see the same object/distance

Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X(); // first sensor (the one with the black dot)
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X(); // second sensor


const float RANGE_MODIFIER = 0.006;
const float RANGE_EXPONENT = 3.0;

void playMelody()
{
  int melody[] = {523, 494, 587, 659, 440, 494, 349};
  int duration[] = {150, 100, 200, 150, 300, 100, 350};

  for (int i = 0; i < 7; i++) {
    tone(PIN_BUZZER, melody[i], duration[i]);
    delay(duration[i] + 50);
  }
}

int correctMeasurements(int * measurements) // takes an array of MEASUREMENTS_COUNT ints, returns one int
{
  int measurements_delta[MEASUREMENTS_COUNT] = {0};

  int sum = 0;
  for (int i = 0; i < MEASUREMENTS_COUNT; i++)
  {
    sum += measurements[i];
  }
  float avarge = (float)sum / (float)MEASUREMENTS_COUNT;
  int max_delta = 0;
  for (int i = 0; i < MEASUREMENTS_COUNT; i++)
  {
    measurements_delta[i] = abs(measurements[i] - avarge);
    max_delta = max(measurements_delta[i], max_delta);
  }
  for (int i = 0; i < MEASUREMENTS_COUNT; i++)
  {
    if (measurements_delta[i] == max_delta)
    {
      measurements[i] = -1;
      break;
    }
  }
  sum = 0;
  for (int i = 0; i < MEASUREMENTS_COUNT; i++)
  {
    int val = measurements[i]!=-1?measurements[i]:0;
    sum += val;
  }
  avarge = (float)sum / ((float)MEASUREMENTS_COUNT - 1.0);
  return avarge;
}

void pinsInit()
{
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT);
  pinMode(PIN_XSHUT_LEFT, OUTPUT);
  digitalWrite(PIN_XSHUT_RIGHT, HIGH);
  digitalWrite(PIN_XSHUT_LEFT, HIGH);
}

void setup()
{
  Serial.begin(115200); // whatever, usb speed anyway

  pinsInit();
  Wire.begin(8, 9);

  playMelody();

  // begin sensors
  digitalWrite(PIN_XSHUT_LEFT, LOW);
  while(!sensorRight.begin(0x30))
  {
    tone(PIN_BUZZER, 2000, 20);
    delay(100);
  }

  digitalWrite(PIN_XSHUT_LEFT, HIGH);
  while(!sensorLeft.begin(0x29))
  {
    tone(PIN_BUZZER, 1000, 10);
    delay(200);
    tone(PIN_BUZZER, 1000, 50);
    delay(1000);
  }

  Serial.println("hello");

  sensorRight.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  sensorRight.startRangeContinuous();
  sensorLeft.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  sensorLeft.startRangeContinuous();

  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
    }
  }

  int rightRange = 0;
  int leftRange = 0;

  int rightMeasurementCount = 0;
  int leftMeasurementCount = 0;

  int rightMeasurements[MEASUREMENTS_COUNT];
  int leftMeasurements[MEASUREMENTS_COUNT];

  unsigned long start_time_ms = millis();

  int finalRange = 0;
  int oldFinalRange = 0;
  while (true)
  {
    if (sensorRight.isRangeComplete())
    {
      rightMeasurements[rightMeasurementCount] = min((int)sensorRight.readRange(), SENSOR_MAX_RANGE_MM);
      rightMeasurementCount += 1;
      if (rightMeasurementCount == MEASUREMENTS_COUNT)
      {
        rightMeasurementCount = 0;
        rightRange = correctMeasurements(rightMeasurements);
        Serial.print("RIGHT ");
        Serial.println(rightRange);
        // long ass line ; and duplicated
        // depreciated cuz doesnt work well
        // will use another alghoritm
        //finalRange = (abs(rightRange - leftRange) < (rightRange * MEASUREMENTS_PROMIXITY_PERCENT + leftRange * MEASUREMENTS_PROMIXITY_PERCENT) / 2.0) ? (rightRange + leftRange) / 2.0 : SENSOR_MAX_RANGE_MM;
        finalRange = min(SENSOR_MAX_RANGE_MM, min(rightRange, leftRange));
      }
    }
    if (sensorLeft.isRangeComplete())
    {
      leftMeasurements[leftMeasurementCount] = min((int)sensorLeft.readRange(), SENSOR_MAX_RANGE_MM);
      leftMeasurementCount += 1;
      if (leftMeasurementCount == MEASUREMENTS_COUNT)
      {
        leftMeasurementCount = 0;
        leftRange = correctMeasurements(leftMeasurements);
        Serial.print("LEFT ");
        Serial.println(leftRange);
        finalRange = min(SENSOR_MAX_RANGE_MM, min(rightRange, leftRange));
      }
    }
    
    if (millis() - pow(finalRange*RANGE_MODIFIER, RANGE_EXPONENT) > start_time_ms)
    {
      Serial.print("final");
      Serial.println(finalRange);
      tone(PIN_BUZZER, 1000, 5);
      delay(1000);
      start_time_ms = millis();
    }
  }
}

void loop()
{

}
