/**
  Name: WorkCounter
  Purpose: Counts workcycles and used energy based on distance, set limits, weight and gravity.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"

class WorkCounter {
    float _highLimit = 0;
    float _lowLimit = 0;
    float _mass = 0;
    float _gravity = 0;
    float _downMotionCoef = 0;
    float _energyCounter = 0;
    int _workCounter = 0;
    bool _upState = false;
  public:
    WorkCounter(float highLimit, float lowLimit, float mass, float gravity, float downMotionCoef);
    void measure(float value);
    int getCounterValue();
    float getEnergyCounter();
};
