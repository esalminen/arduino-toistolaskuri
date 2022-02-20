/**
  Name: SonarSensor
  Purpose: Retrieves distance from HC-SR04 sensor

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include "Arduino.h"

class SonarSensor {
    int _gndPin = 0;
    int _echoPin = 0;
    int _trigPin = 0;
    int _vccPin = 0;
    float _maxDistance = 0;
    float _minDistance = 0;
    float _distance = 0;
  public:
    SonarSensor(int gndPin, int echoPin, int trigPin, int vccPin, float maxDistance, float minDistance);

    /**
      Initialize sensor
    */
    void init();

    /**
       Measure distance. Value -1.0 is stored in distance if sensor is unable to measure distance
    */
    void measure();

    /**
       Get measured distance
       @param return  Return distance as cm
    */
    float getMeasurement();
    
    /**
      Prints measurement data to serial port with Serial.print() command
    */
    void printData();
};
