/**
  Name: AccToSpeed
  Purpose: Calculates current speed from given accelerometer values.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"
#include "AccToSpeed.h"

AccToSpeed::AccToSpeed(float aFreq, float vFreq)
{
  _axyzFilterFreq = aFreq;
  _vxyzFilterFreq = vFreq;
  _axFilter = new Filter(aFreq);
  _ayFilter = new Filter(aFreq);
  _azFilter = new Filter(aFreq);
  _vxFilter = new Filter(vFreq);
  _vyFilter = new Filter(vFreq);
  _vzFilter = new Filter(vFreq);

}

void AccToSpeed::setInput(float ax, float ay, float az, float inputInterval)
{
  _ax = ax;
  _ay = ay;
  _az = az;

  _axFilter->setInput(_ax, inputInterval);
  _ayFilter->setInput(_ay, inputInterval);
  _azFilter->setInput(_az, inputInterval);

  _axAvg = _axFilter->getOutput();
  _ayAvg = _ayFilter->getOutput();
  _azAvg = _azFilter->getOutput();

  _axOut = _ax - _axAvg;
  _ayOut = _ay - _ayAvg;
  _azOut = _az - _azAvg;

  _vx = _vx + _axOut * inputInterval;
  _vy = _vy + _ayOut * inputInterval;
  _vz = _vz + _azOut * inputInterval;

  _vxFilter->setInput(_vx, inputInterval);
  _vyFilter->setInput(_vy, inputInterval);
  _vzFilter->setInput(_vz, inputInterval);

  _vxAvg = _vxFilter->getOutput();
  _vyAvg = _vyFilter->getOutput();
  _vzAvg = _vzFilter->getOutput();

  _vxOut = _vx - _vxAvg;
  _vyOut = _vy - _vyAvg;
  _vzOut = _vz - _vzAvg;
}

float AccToSpeed::getOutputMax()
{
  return max(_vxOut, max(_vyOut, _vzOut));
}

float AccToSpeed::getOutputMin()
{
  return min(_vxOut, min(_vyOut, _vzOut));
}

void AccToSpeed::printAccData()
{
  Serial.print("ax[m/s^2]:");
  Serial.print(_ax); Serial.print(" ");
    Serial.print("axAvg[m/s^2]:");
  Serial.print(_axAvg); Serial.print(" ");
    Serial.print("axOut[m/s^2]:");
  Serial.print(_axOut); Serial.print(" ");
  //  Serial.print("ayOut[m/s^2]:");
  //  Serial.print(_ayOut); Serial.print(" ");
  //  Serial.print("azOut[m/s^2]:");
  //  Serial.print(_azOut); Serial.print(" ");
}

void AccToSpeed::printVelocityData()
{
  Serial.print("vx[m/s^2]:");
  Serial.print(_vx*10); Serial.print(" ");
  Serial.print("vxAvg[m/s^2]:");
  Serial.print(_vxAvg*10); Serial.print(" ");
  Serial.print("vxOut[m/s^2]:");
  Serial.print(_vxOut*10); Serial.print(" ");
  //  Serial.print("vyOut[m/s^2]:");
  //  Serial.print(_vyOut); Serial.print(" ");
  //  Serial.print("vzOut[m/s^2]:");
  //  Serial.print(_vzOut); Serial.print(" ");
}
