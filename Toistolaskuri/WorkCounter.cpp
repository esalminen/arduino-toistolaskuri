/**
  Name: WorkCounter
  Purpose: Counts workcycles and used energy based on distance, set limits, weight and gravity.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"
#include "WorkCounter.h"

WorkCounter::WorkCounter(float highLimit, float lowLimit, float mass, float gravity, float downMotionCoef)
{
  _highLimit = highLimit;
  _lowLimit = lowLimit;
  _mass = mass;
  _gravity = gravity;
  _downMotionCoef = downMotionCoef;
}

void WorkCounter::measure(float distance)
{
  float derivative = 0.0;

  if (abs(distance - _prevDistance) > 0.02)
  {
    float deltaTime = (millis() - _prevTime) / 1000.0;
    float deltaDistance = distance - _prevDistance;
    float derivative = deltaDistance / deltaTime;
    _prevTime = millis();
    _prevDistance = distance;

    if (derivative < 0)
    {
      _energyCounter += _mass * _gravity * abs(deltaDistance) * _downMotionCoef;
    }
    else if (derivative > 0)
    {
      _energyCounter += _mass * _gravity * deltaDistance;
    }
  }

  if (!_upState && (distance) >= _highLimit) {
    _workCounter++; // Lisätään toistojen määrää yhdellä
    _upState = true;
  }

  if (_upState && (distance) <= _lowLimit) {
    _upState = false;
  }
}

void WorkCounter::measure(float velocity, bool dummy)
{
  
}

int WorkCounter::getCounterValue()
{
  return _workCounter;
}

float WorkCounter::getEnergyCounter()
{
  return _energyCounter;
}

void WorkCounter::printData()
{
  Serial.print("workCounter[cycles]:");
  Serial.print(_workCounter); Serial.print(" ");
  Serial.print("energyCounter[J]:");
  Serial.print(_energyCounter); Serial.print(" ");
}
