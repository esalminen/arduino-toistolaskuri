/**
  Name: WorkCounter
  Purpose: Counts workcycles and used energy based on distance, set limits, weight and gravity.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"

class WorkCounter {
    float _prevDistance = 0;
    float _highLimit = 0;
    float _lowLimit = 0;
    float _mass = 0;
    float _gravity = 0;
    float _downMotionCoef = 0;
    float _energyCounter = 0;
    int _workCounter = 0;
    bool _upState = false;
    unsigned long _prevTime = 0;
  public:
    WorkCounter(float highLimit, float lowLimit, float mass, float gravity, float downMotionCoef);
    /**
       Measure work counts
       @param distance  distance from sensor in cm
    */
    void measure(float distance);

    /**
           Measure work counts
           @param ax  x-axis acceleration from sensor
           @param ay  y-axis -..-
           @param az  z-axis -..-
    */
    void measure(float ax, float ay, float az);

    /**
       Get work counter current value
       @param return  work counter value
    */
    int getCounterValue();

    /**
       Get used energy counter value
       @param return used energy counter value
    */
    float getEnergyCounter();

    /**
      Prints measurement data to serial port with Serial.print() command
    */
    void printData();
};
