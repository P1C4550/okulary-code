#include <Arduino.h>
#include "WiFi.h"
#include "Adafruit_VL53L0X.h"

#include "wifi_config.h"

#define PIN_SDA_PIN 8 // connected HIGH side to builtin LED
#define PIN_SCL_PIN 9
#define PIN_BUZZER 10
#define PIN_XSHUT_RIGHT 2
#define PIN_XSHUT_LEFT 3

const double RANGE_MODIFIER = 0.006;
const double RANGE_EXPONENT = 3.0;
const int MEASUREMENTS_LEN = 3;

Adafruit_VL53L0X sensor_right = Adafruit_VL53L0X(); // first sensor (the one with the black dot)
Adafruit_VL53L0X sensor_left = Adafruit_VL53L0X(); // second sensor

void playMelody()
{
  int melody[] = {523, 494, 587, 659, 440, 494, 349};
  int duration[] = {150, 100, 200, 150, 300, 100, 350};

  for (int i = 0; i < 7; i++) {
    tone(PIN_BUZZER, melody[i], duration[i]);
    delay(duration[i] + 50);
  }
}

void pins_init()
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

  pins_init();
  Wire.begin(8, 9);

  playMelody();

  // begin sensors

  //  // sensor 1 begin
  digitalWrite(PIN_XSHUT_LEFT, LOW);
  while(!sensor_right.begin(0x30))
  {
    tone(PIN_BUZZER, 3000, 10);
    delay(50);
  }

  digitalWrite(PIN_XSHUT_LEFT, HIGH);
  while(!sensor_right.begin(0x29))
  {
    tone(PIN_BUZZER, 3000, 10);
    delay(20);
    tone(PIN_BUZZER, 3000, 10);
    delay(50);
  }

  sensor_right.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  sensor_right.startRangeContinuous();
  sensor_left.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  sensor_left.startRangeContinuous();

  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
    }
  }
}
 
void loop()
{

}
