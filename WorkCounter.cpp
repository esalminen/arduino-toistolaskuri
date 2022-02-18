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

void WorkCounter::measure(float value)
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
