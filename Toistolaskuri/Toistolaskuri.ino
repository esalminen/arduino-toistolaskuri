/**
  Name: Toistolaskuri
  Purpose: Workcycle counter based on distance and acceleration measurements.

  @author Esa Salminen
  @version 1.0 18.2.2022
*/

#include <MsTimer2.h>
#include <util/atomic.h>
#include "AccSensor.h"
#include "AccToSpeed.h"
#include "SonarSensor.h"
#include "WorkCounter.h"
#include "Filter.h"



// Common program variables
bool firstProgramCycle = true;

// Time variables
unsigned long currentTime = 0;
int interruptInterval = 5; // [ms]

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
AccToSpeed *accToSpeed;
SonarSensor *sonarSensor;
float distance = 0;
volatile float filteredDistance = 0;
volatile float minVelocity = 0;
volatile float maxVelocity = 0;
void setup() {

  filterDistance = new Filter(2 * PI);

  workCounter = new WorkCounter(highLimit, lowLimit, mass, gravity, downwardMotionCoef);

  accSensor = new AccSensor(0x68);
  accSensor->init(0, 0, 0);
  accSensor->calibrateAcc(0.0006, -0.4275, 0.0006, 0.0390, 0.0006, -0.2623); // calibrate sensor with equation coefficients

  accToSpeed = new AccToSpeed(0.3, 0.6);

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

  accToSpeed->setInput(measuredData.axCalibrated, measuredData.ayCalibrated, measuredData.azCalibrated, interruptInterval / 1000.0);
}

void loop() {

  // Get running time
  currentTime = millis();

  // Measure acceleration
  accSensor->measure();
  measuredData = accSensor->getMeasurements();

  // Measure distance
  //sonarSensor->measure();

  // Workcounter calc from distance. Calculated in interrupt protected atomic
  // block so that interrupt cannot change filteredDistance while measure is called.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    distance = sonarSensor->getMeasurement();
    if (firstProgramCycle) filterDistance->setOutput(distance);
    workCounter->measure(filteredDistance / 100.0);
    minVelocity = accToSpeed->getOutputMin() * 10.0;
    maxVelocity = accToSpeed->getOutputMax() * 10.0;
  }

  printSerialPlotter();
  //printSerialMonitor();

  //Small delay before next cycle
  delay(2);
  firstProgramCycle = false;
}

void printSerialPlotter()
{
  //Serial.print("distance[cm]:");
  //Serial.print(filteredDistance); Serial.print(" ");
  //sonarSensor->printData();
  //accSensor->printData();
//  Serial.print("minV[m/s]:");
//  Serial.print(minVelocity); Serial.print(" ");
//  Serial.print("maxV[m/s]:");
//  Serial.print(maxVelocity); Serial.print(" ");
  //accToSpeed->printAccData();
  accToSpeed->printVelocityData();
  //workCounter->printData();
  Serial.println();
}

void printSerialMonitor()
{
  Serial.print("time[ms]:");
  Serial.print(currentTime); Serial.print(" ");
  Serial.print("distance[cm]:");
  Serial.print(filteredDistance); Serial.print(" ");
  sonarSensor->printData();
  accSensor->printData();
  Serial.println();
}
