/**
  Name: AccSensor
  Purpose: Retrieves acceleration, angular velocity and temperature information from GY-521 sensor via I2C bus

  @author Esa Salminen
  @version 1.0 18.2.2022
*/
#ifndef ACCSENSOR_H
#define ACCSENSOR_H
#include "Arduino.h"

struct AccSensorMeasureData {
  int16_t axRaw;
  int16_t ayRaw;
  int16_t azRaw;
  float axCalibrated;
  float ayCalibrated;
  float azCalibrated;
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;
  float gxCalibrated;
  float gyCalibrated;
  float gzCalibrated;
  int16_t tempRaw;
  float tempCalibrated;
};

class AccSensor {
    int _address = 0;
    int16_t _acx = 0, _acy = 0, _acz = 0;
    int16_t _gyx = 0, _gyy = 0, _gyz = 0;
    int16_t _temp = 0;
    float _xa = 0, _xb = 0, _ya = 0, _yb = 0, _za = 0, _zb = 0;
    AccSensorMeasureData _measureData;
  public:
    AccSensor(int address);

    /**
       Set calibration equation factors to correct output results
       @param xa  X-axis coefficient
       @param xb  X-axis constant
       @param ya  Y-axis coefficient
       @param yb  Y-axis constant
       @param za  Z-axis coefficient
       @param zb  Z-axis constant
    */
    void calibrateAcc(float xa, float xb, float ya, float yb, float za, float zb);
    void calibrateGyro(); // TODO
    void calibrateTemp(); // TODO

    /**
       Initialize sensor with filter setting, gyro setting and acc setting
       Possible filter settings:
       0 = lowpass threshold 260 Hz, delay 0 ms (default)
       1 = 184 Hz, 2.0 ms
       2 = 94 Hz, 3.0 ms
       3 = 44 Hz, 4.9 ms
       4 = 21 Hz, 8.5 ms
       5 = 10 Hz, 13.8 ms
       6 = 5 Hz, 19.0 ms
       Possible gyro settings:
       0 = +- 250 °/s (default)
       1 = +- 500 °/s
       2 = +- 1000 °/s
       3 = +- 2000 °/s
       Possible acc settings:
       0 = +- 2g (default)
       1 = +- 4g
       2 = +- 8g
       3 = +-16g
       @param filterConfig  Filter parameter
       @param gyroConfig    Gyro parameter
       @param accConfig     Acc parameter
    */
    void init(byte filterConfig = 0, byte gyroConfig = 0, byte accConfig = 0);

    /**
       Measure acceleration, gyroscope and temperature into object memory as calibrated and raw values
    */
    void measure();

    /**
       Get measured data as a struct
    */
    AccSensorMeasureData getMeasurements();

    /**
       Prints measurement data to serial port with Serial.print() command
    */
    void printData();
};
#endif
