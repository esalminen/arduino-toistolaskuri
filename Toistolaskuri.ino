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
    Filter(float filterFreqRad)
    {
      _filterFreqRad = filterFreqRad;
    }

    void setInput(float value, float cycleInterval)
    {
      _outputValue = _outputValue + _filterFreqRad * cycleInterval * (value - _outputValue);
    }

    void setOutput(float value)
    {
      _outputValue = value;
    }

    float getOutput()
    {
      return _outputValue;
    }
};

// Common program variables
bool firstProgramCycle = true;

// Time variables
unsigned long currentTime = 0;
unsigned long prevTime = 0;
float deltaTime = 0;

// Sonar sensor parameters
int gndPin = 11;
int echoPin = 10;
int trigPin = 9;
int vccPin = 8;
float maxDistance = 100.0;
float minDistance = 0.0;

// Workcounter parameters
float highLimit = 0.4; // [m]
float lowLimit = 0.2; // [m]
float mass = 5.5; // [kg]
float gravity = 9.81; // [m/s^2]
float downwardMotionCoef = 0.6;

const float SNICKERS_ENERGIA = 481.0 ; // Patukan energia [kcal]
const float KEHON_HYOTYSUHDE = 0.15; // Keho kuluttaa energiaa työtä tehdessä 15 % hyötysuhteella

Filter *filterDistance;
WorkCounter *workCounter;
AccSensor *accSensor;
AccSensorMeasureData measuredData;
SonarSensor *sonarSensor;
float distance = 0;
volatile float filteredDistance = 0;

void setup() {

  filterDistance = new Filter(10.0);

  workCounter = new WorkCounter(highLimit, lowLimit, mass, gravity, downwardMotionCoef);

  accSensor = new AccSensor(0x68);
  accSensor->init(4, 0, 0);
  accSensor->calibrateAcc(0.0006, -0.4275, 0.0006, 0.0390, 0.0006, -0.2623); // calibrate sensor with equation coefficients

  sonarSensor = new SonarSensor(gndPin, echoPin, trigPin, vccPin, maxDistance, minDistance);
  sonarSensor->init();

  MsTimer2::set(10, interrupt);
  MsTimer2::start();

  Serial.begin (19200);
  while (Serial.available() != 0)
  {
    delay(1); // small delay before entering main loop
  }
}

void interrupt()
{
  filterDistance->setInput(distance, 0.01);
  filteredDistance = filterDistance->getOutput();
}

void loop() {

  // Get cycle time and count cycle interval
  prevTime = currentTime;
  currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000.0;

  // Measure acceleration
  accSensor->measure();
  measuredData = accSensor->getMeasurements();

  // Measure distance
  sonarSensor->measure();
  distance = sonarSensor->getMeasurement();
  if (firstProgramCycle) filterDistance->setOutput(distance);

  // Workcounter calc from distance. Calculated in interrupt protected atomic
  // block so that interrupt cannot change filteredDistance while measure is called.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
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
  sonarSensor->printData();
  accSensor->printData();
  workCounter->printData();
  Serial.println();
}

void printSerialMonitor() 
{
  Serial.print("time[ms]:");
  Serial.print(currentTime); Serial.print(" ");
  sonarSensor->printData();
  accSensor->printData();
  workCounter->printData();
  Serial.println();
}
