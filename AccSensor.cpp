/**
  Name: AccSensor
  Purpose: Retrieves acceleration, angular velocity and temperature information from GY-521 sensor.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"
#include "AccSensor.h"
#include <Wire.h>

AccSensor::AccSensor(int address)
{
  _address = address;
}

void AccSensor::calibrateAcc(float xa, float xb, float ya, float yb, float za, float zb)
{
  _xa = xa;
  _xb = xb;
  _ya = ya;
  _yb = yb;
  _za = za;
  _zb = zb;
}

void AccSensor::init(byte filterConfig, byte gyroConfig, byte accConfig)
{
  // Wakes MPU-6050 sensor
  Wire.begin();
  Wire.beginTransmission(_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(false);

  // Filter configuration in register 0x1A
  Wire.beginTransmission(_address);
  Wire.write(0x1A);
  Wire.write(filterConfig);
  Wire.endTransmission(false);

  // Gyro configuration in register 0x1B
  Wire.beginTransmission(_address);
  Wire.write(0x1B);
  Wire.write(gyroConfig << 3);
  Wire.endTransmission(false);

  // Acc configuration in register 0x1C
  Wire.beginTransmission(_address);
  Wire.write(0x1C);
  Wire.write(accConfig << 3);
  Wire.endTransmission(true);
}

void AccSensor::measure()
{
  Wire.beginTransmission(_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(_address, 14, true);

  _acx = Wire.read() << 8 | Wire.read();
  _acy = Wire.read() << 8 | Wire.read();
  _acz = Wire.read() << 8 | Wire.read();
  _temp = Wire.read() << 8 | Wire.read();
  _gyx = Wire.read() << 8 | Wire.read();
  _gyy = Wire.read() << 8 | Wire.read();
  _gyz = Wire.read() << 8 | Wire.read();

  _measureData.axRaw = _acx;
  _measureData.ayRaw = _acy;
  _measureData.azRaw = _acz;
  _measureData.axCalibrated = _xa * _acx + _xb;
  _measureData.ayCalibrated = _ya * _acy + _yb;
  _measureData.azCalibrated = _za * _acz + _zb;

  _measureData.gxRaw = _gyx;
  _measureData.gyRaw = _gyy;
  _measureData.gzRaw = _gyz;
  _measureData.gxCalibrated = _gyx * 1.0;
  _measureData.gyCalibrated = _gyy * 1.0;
  _measureData.gzCalibrated = _gyz * 1.0;

  /*temperature calculation
    tx = Tmp + tcal;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //fahrenheit*/
  _measureData.tempRaw = _temp;
  _measureData.tempCalibrated = _temp / 340.0 + 36.53;  // equation for temperature in °C
}

AccSensorMeasureData AccSensor::getMeasurements()
{
  return _measureData;
}

void AccSensor::printData()
{
  Serial.print("axCalibrated[m/s^2]:");
  Serial.print(_measureData.axCalibrated);Serial.print(" ");
  Serial.print("ayCalibrated[m/s^2]:");
  Serial.print(_measureData.ayCalibrated);Serial.print(" ");
  Serial.print("azCalibrated[m/s^2]:"); 
  Serial.print(_measureData.azCalibrated);Serial.print(" ");
}
