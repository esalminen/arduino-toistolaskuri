/**
  Name: WorkCounter
  Purpose: Counts workcycles and used energy based on distance, set limits, weight and gravity.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"
#include "WorkCounter.h"

WorkCounter::WorkCounter(float highLimit, float lowLimit, float mass, float gravity, float downMotionCoef, float highVelocityLimit, float lowVelocityLimit)
{
  _highLimit = highLimit;
  _lowLimit = lowLimit;
  _highVelocityLimit = highVelocityLimit;
  _lowVelocityLimit = lowVelocityLimit;
  _mass = mass;
  _gravity = gravity;
  _downMotionCoef = downMotionCoef;
}

void WorkCounter::measure(float paramValue, int mode)
{
  if (mode == 0)
  { // distance mode
    float derivative = 0.0;
    // if distance change is greate than 2 cm, calculate new energy consumption
    if (abs(paramValue - _prevDistance) > 0.02)
    {
      float deltaTime = (millis() - _prevTime) / 1000.0;
      float deltaDistance = paramValue - _prevDistance;
      float derivative = deltaDistance / deltaTime;
      _prevTime = millis();
      _prevDistance = paramValue;

      // negative rate of change means we are lowering the weight
      if (derivative < 0)
      {
        _energyCounter += _mass * _gravity * abs(deltaDistance) * _downMotionCoef;
      }

      // positive rate of change means we are lifting the weight
      else if (derivative > 0)
      {
        _energyCounter += _mass * _gravity * deltaDistance;
      }
    }

    if (!_upState && (paramValue) >= _highLimit) {
      _workCounter++;
      _upState = true;
    }

    if (_upState && (paramValue) <= _lowLimit) {
      _upState = false;
    }
  }
  else if (mode == 1)
  { // velocity mode
    if (!_upState && (paramValue) >= _highVelocityLimit) {
      _velocityRepetition++;
      _upState = true;
    }

    if (_upState && (paramValue) <= _lowVelocityLimit) {
      _upState = false;
    }
    _workCounter = _velocityRepetition / 2;
  }
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
  Serial.print("upState:");
  Serial.print(_upState * 10); Serial.print(" ");
  Serial.print("velRep[kpl]:");
  Serial.print(_velocityRepetition); Serial.print(" ");
}
