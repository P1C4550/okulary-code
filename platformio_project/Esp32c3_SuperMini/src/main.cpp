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

#define SENSOR_MAX_RANGE_MM (short)2500

Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X(); // first sensor (the one with the black dot)
Adafruit_VL53L0X sensorLeft = Adafruit_VL53L0X(); // second sensor

void playMelody()
{
  int melody[] = {523, 494, 587, 659, 440, 494, 349};
  int duration[] = {150, 100, 200, 150, 300, 100, 350};

  for (int i = 0; i < 7; i++) {
    tone(PIN_BUZZER, melody[i], duration[i]);
    delay(duration[i] + 50);
  }
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
  Serial.begin(9600); // whatever, usb speed anyway

  pinsInit();
  Wire.begin(8, 9);

  playMelody();

  // begin sensors

  //  // sensor 1 begin
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

  short rightRange = 0;
  short leftRange = 0;
  while (true)
  {
    if (sensorRight.isRangeComplete())
    {
      rightRange = min((short)sensorRight.readRange(), SENSOR_MAX_RANGE_MM);
    }
    if (sensorLeft.isRangeComplete())
    {
      leftRange = min((short)sensorLeft.readRange(), SENSOR_MAX_RANGE_MM);
    }
    // tutaj pipanie i matma cala
  }
}

void loop()
{

}
