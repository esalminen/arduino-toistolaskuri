/**
  Name: SonarSensor
  Purpose: Retrieves distance from HC-SR04 sensor

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"
#include "SonarSensor.h"

SonarSensor::SonarSensor(int gndPin, int echoPin, int trigPin, int vccPin, float maxDistance, float minDistance)
{
  _gndPin = gndPin;
  _echoPin = echoPin;
  _trigPin = trigPin;
  _vccPin = vccPin;
  _maxDistance = maxDistance;
  _minDistance = minDistance;
}

void SonarSensor::init()
{
  // Define pins
  pinMode(_gndPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  pinMode(_trigPin, OUTPUT);
  pinMode(_vccPin, OUTPUT);
  
  // Set supplyvoltage levels
  digitalWrite(_vccPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(_gndPin, LOW);
  delayMicroseconds(2);
}

void SonarSensor::measure()
{
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  unsigned long duration = pulseIn(_echoPin, HIGH); // Sound travelling time back and forth in µs. Sound travels approx 0.034 cm in µs
  _distance = duration * 0.034 / 2.0; // Divide with 2.0 because duration includes travelling to and from target
  if (_distance >= _maxDistance || _distance <= _minDistance) _distance = -1.0; // Set distance to -1.0 if measurement fails
}

float SonarSensor::getMeasurement()
{
  return _distance;
}

void SonarSensor::printData()
{
  Serial.print("distance[cm]:");
  Serial.print(_distance);Serial.print(" ");
}
