/**
  Name: Filter
  Purpose: 1st order lowpass filter.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/
#ifndef FILTER_H
#define FILTER_H
#include "Arduino.h"

class Filter {
    float _filterFreqRad = 0;
    float _outputValue = 0;
  public:

    /**
        Filter constructor
        @param filterFreqRad  Filtering frequency in rad. 2 * PI = 1 Hz
        @param cycleInterval  Input signal read interval in seconds
    */
    Filter(float filterFreqRad);

    /**
      Set filter input signal
      @param value          Input signal
      @param cycleInterval  Input signal read interval in seconds
    */
    void setInput(float value, float cycleInterval);

    /**
      Set filter output signal. Used to make quick step change in filter output
      @param value          Output signal
    */
    void setOutput(float value);

    /**
      Set filter output signal. Used to make quick step change in filter output
      @param return         Get filter curren output signal
    */
    float getOutput();
};
#endif
