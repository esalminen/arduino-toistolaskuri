/**
  Name: Filter
  Purpose: 1st order lowpass filter.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"
#include "Filter.h"

Filter::Filter(float filterFreqRad)
{
  _filterFreqRad = filterFreqRad;
}

void Filter::setInput(float value, float cycleInterval)
{
  _outputValue = _outputValue + _filterFreqRad * cycleInterval * (value - _outputValue);
}


void Filter::setOutput(float value)
{
  _outputValue = value;
}


float Filter::getOutput()
{
  return _outputValue;
}
