/*
  MQ5.cpp - Library for the MQ5 gas sensor.
  Reference value needs to be adjusted to work for the teensy.
  SPECS:
  Library Size: 9kB (3%) ROM, 3.4kB (5%) RAM
  Each Initialization: 2.6kB (1%) RAM
*/

#include "Arduino.h"
#include "MQ5.h"
float CalValue = 0;

MQ5::MQ5()
{

}

void MQ5::attachPin(int pin)
{
  pinMode(pin, INPUT);
  _pin = pin;
}

int MQ5::RawAnalog()
{
  return analogRead(_pin);
}

float MQ5::PPM()
{
  float sensor_volt;
  float RS_gas;
  float ratio;
  int sensorValue = analogRead(_pin);
  sensor_volt = (float)sensorValue / 1024 * 5.0;
  RS_gas = (5.0 - sensor_volt) / sensor_volt;
  ratio = RS_gas / CalValue;
  return ratio;
}

void MQ5::Cal()
{
  float sensor_volt;
  float RS_air;
  float R0;
  float sensorValue = 0;
  for (int i = 0; i < 100; i++)
  {
    sensorValue = sensorValue + analogRead(_pin);
  }
  sensorValue = sensorValue / 100.0;
  sensor_volt = sensorValue / 1024 * 5.0;
  RS_air = (5.0 - sensor_volt) / sensor_volt;
  R0 = RS_air / 6.5;
  Serial.println(R0);
  CalValue = R0;
}
