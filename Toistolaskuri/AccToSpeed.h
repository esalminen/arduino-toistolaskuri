/**
  Name: AccToSpeed
  Purpose: Calculates current speed from given accelerometer values.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/
#ifndef ACCTOSPEED_H
#define ACCTOSPEED_H
#include "Arduino.h"
#include "Filter.h"

class AccToSpeed {
    Filter *_axFilter;
    Filter *_ayFilter;
    Filter *_azFilter;
    Filter *_vxFilter;
    Filter *_vyFilter;
    Filter *_vzFilter;
    float _axyzFilterFreq = 0;
    float _vxyzFilterFreq = 0;
    float _ax = 0;
    float _ay = 0;
    float _az = 0;
    float _vx = 0;
    float _vy = 0;
    float _vz = 0;
    float _axAvg = 0;
    float _ayAvg = 0;
    float _azAvg = 0;
    float _vxAvg = 0;
    float _vyAvg = 0;
    float _vzAvg = 0;
    float _axOut = 0;
    float _ayOut = 0;
    float _azOut = 0;
    float _vxOut = 0;
    float _vyOut = 0;
    float _vzOut = 0;
    float _vxyzOut = 0;
  public:
    AccToSpeed(float aFreq, float vfreq);
    void init(float ax, float ay, float az);
    void setInput(float ax, float ay, float az, float inputInterval);
    float getOutput();
    void printAccData();
    void printVelocityData();
};
#endif
