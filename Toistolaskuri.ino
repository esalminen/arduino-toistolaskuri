/**
  Name: Toistolaskuri
  Purpose: Workcycle counter based on distance and acceleration measurements.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include <MsTimer2.h>
#include <util/atomic.h>
#include "AccSensor.h"
#include "SonarSensor.h"
#include "WorkCounter.h"

class Filter {
    float _filterFreqRad = 0;
    float _outputValue = 0;
  public:

    /**
        Filter constructor
        @param filterFreqRad  Filtering frequency in rad. 2 * PI = 1 Hz
        @param cycleInterval  Input signal read interval in seconds
    */
    Filter(float filterFreqRad)
    {
      _filterFreqRad = filterFreqRad;
    }

    /**
      Set filter input signal
      @param value          Input signal
      @param cycleInterval  Input signal read interval in seconds
    */
    void setInput(float value, float cycleInterval)
    {
      _outputValue = _outputValue + _filterFreqRad * cycleInterval * (value - _outputValue);
    }

    /**
      Set filter output signal. Used to make quick step change in filter output
      @param value          Output signal
    */
    void setOutput(float value)
    {
      _outputValue = value;
    }

    /**
      Set filter output signal. Used to make quick step change in filter output
      @param return         Get filter curren output signal
    */
    float getOutput()
    {
      return _outputValue;
    }
};

// Common program variables
bool firstProgramCycle = true;

// Time variables
unsigned long currentTime = 0;
int interruptInterval = 10; // [ms]

// Sonar sensor parameters
int gndPin = 11;
int echoPin = 10;
int trigPin = 9;
int vccPin = 8;
float maxDistance = 100.0; // [cm]
float minDistance = 0.0; // [cm]

// Workcounter parameters
float highLimit = 0.4; // [m]
float lowLimit = 0.2; // [m]
float mass = 5.5; // [kg]
float gravity = 9.81; // [m/s^2]
float downwardMotionCoef = 0.6;

Filter *filterDistance;
WorkCounter *workCounter;
AccSensor *accSensor;
AccSensorMeasureData measuredData;
SonarSensor *sonarSensor;
float distance = 0;
volatile float filteredDistance = 0;

void setup() {

  filterDistance = new Filter(2 * PI);

  workCounter = new WorkCounter(highLimit, lowLimit, mass, gravity, downwardMotionCoef);

  accSensor = new AccSensor(0x68);
  accSensor->init(4, 0, 0);
  accSensor->calibrateAcc(0.0006, -0.4275, 0.0006, 0.0390, 0.0006, -0.2623); // calibrate sensor with equation coefficients

  sonarSensor = new SonarSensor(gndPin, echoPin, trigPin, vccPin, maxDistance, minDistance);
  sonarSensor->init();

  MsTimer2::set(interruptInterval, interrupt);
  MsTimer2::start();

  Serial.begin (19200);
  while (Serial.available() != 0)
  {
    delay(1); // small delay before entering main loop
  }
}

void interrupt()
{
  filterDistance->setInput(distance, interruptInterval / 1000.0);
  filteredDistance = filterDistance->getOutput();
}

void loop() {

  // Get running time
  currentTime = millis();

  // Measure acceleration
  accSensor->measure();
  measuredData = accSensor->getMeasurements();

  // Measure distance
  sonarSensor->measure();

  // Workcounter calc from distance. Calculated in interrupt protected atomic
  // block so that interrupt cannot change filteredDistance while measure is called.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    distance = sonarSensor->getMeasurement();
    if (firstProgramCycle) filterDistance->setOutput(distance);
    workCounter->measure(filteredDistance / 100.0);
  }

  printSerialPlotter();
  //printSerialMonitor();

  //Small delay before next cycle
  delay(2);
  firstProgramCycle = false;
}

void printSerialPlotter()
{
  Serial.print("distance[cm]:");
  Serial.print(filteredDistance); Serial.print(" ");
  //sonarSensor->printData();
  //accSensor->printData();
  workCounter->printData();
  Serial.println();
}

void printSerialMonitor()
{
  Serial.print("time[ms]:");
  Serial.print(currentTime); Serial.print(" ");
  Serial.println();
}
