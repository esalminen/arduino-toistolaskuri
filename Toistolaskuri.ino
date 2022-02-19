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

//float energialaskuri = 0; // Kulutettu energia jouleina [J]
//float kulutettuEnergiaKcal = 0; // Kulutettu energia kilokaloreina [kcal]
//float kulutetutSnickersit = 0; // Kulutetut Snickers-patukat
//float muutosnopeus = 0.0; // Etäisyyden muutosnopeus cm/s
//float smoothedMuutosnopeus = 0.0; // Etäisyyden muutosnopeus cm/s filtteröitynä
//float prevDistance = 0.0; // Edellisen ohjelmakierron etäisyys
//float deltaDistance = 0.0; // Edellisen ohjelmakierron ja nykyisen ohjelmakierron erotus

Filter *filterDistance;
WorkCounter *workCounter;
AccSensor *accSensor;
AccSensorMeasureData measuredData;
SonarSensor *sonarSensor;
float distance = 0;
volatile float filteredDistance = 0;

void setup() {

  filterDistance = new Filter(0.25);

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
  filterDistance->setInput(distance, 0.5);
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
  if(firstProgramCycle) filterDistance->setOutput(distance);

  // Workcounter calc from distance. Calculated in interrupt protected atomic 
  // block so that interrupt cannot change filteredDistance while measure is called.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    workCounter->measure(filteredDistance / 100.0);
  }


  //  axAvg = axAvg + Ts * aFilterRad * ( ax - axAvg);
  //  ayAvg = ayAvg + Ts * aFilterRad * ( ay - ayAvg);
  //  azAvg = azAvg + Ts * aFilterRad * ( az - azAvg);
  //
  //  axPlot = ax - axAvg;
  //  ayPlot = ay - ayAvg;
  //  azPlot = az - azAvg;
  //
  //  vx = vx + ((aika - prevTime) / 1000.0) * axPlot;
  //  vy = vy + ((aika - prevTime) / 1000.0) * ayPlot;
  //  vz = vz + ((aika - prevTime) / 1000.0) * azPlot;
  //
  //  vxAvg = vxAvg + Ts * vFilterRad * ( vx - vxAvg);
  //  vyAvg = vyAvg + Ts * vFilterRad * ( vy - vyAvg);
  //  vzAvg = vzAvg + Ts * vFilterRad * ( vz - vzAvg);
  //
  //  vxPlot = vx - vxAvg;
  //  vyPlot = vy - vyAvg;
  //  vzPlot = vz - vzAvg;
  //
  //  vxyz = sqrt(vxPlot * vxPlot + vyPlot * vyPlot + vzPlot * vzPlot);
  //  vxyzPlot = vxyz;

  // Lasketaan monta snickersiä on palanut
  //  kulutettuEnergiaKcal = energialaskuri / 4.1868 / 1000.0;
  //  kulutetutSnickersit = (kulutettuEnergiaKcal / KEHON_HYOTYSUHDE) / SNICKERS_ENERGIA;



  printSerialPlotter();
  //printSerialMonitor();
  //printCalibrationData();

  //Viive ennen seuraavaa kierrosta niin dataa tulee kohtuullisella tahdilla
  // Jos mittaa hirmuisen nopeasti niin numerinen derivaatta näyttää kohinaisemmalta
  // vaikka siinä toki olisi tietoa enemmän!
  delay(2);
  firstProgramCycle = false;
}


void printSerialPlotter() {
  //    Serial.print("aika[s]:");
  //    Serial.print(aika); Serial.print(" ");
  //  Serial.print("ylhaalla:");
  //  Serial.print(ylhaalla * 30); Serial.print(" "); // Kerrotaan 30 jotta saadaan askelta vähän isommaksi.
  //  Serial.print("etaisyys[cm]:");
  //  Serial.print(distance); Serial.print(" ");
  //  Serial.print("tasoitettuEtaisyys[cm]:");
  //  Serial.print(smoothedDistance); Serial.print(" ");
  //  Serial.print("min[cm]:");
  //  Serial.print(minDistance); Serial.print(" ");
  //  Serial.print("max[cm]:");
  //  Serial.print(maxDistance); Serial.print(" ");
    Serial.print("toistomäärä[kpl]:");
    Serial.print(workCounter->getCounterValue()*10); Serial.print(" ");
  //  Serial.print("muutosnopeus_ds/dt:");
  //  Serial.print(smoothedMuutosnopeus); Serial.print(" ");
  Serial.print("energialaskuri[J]:");
  Serial.print(workCounter->getEnergyCounter()); Serial.print(" ");
  //  Serial.print("energia[kcal]:");
  //  Serial.print(kulutettuEnergiaKcal); Serial.print(" ");
  //  Serial.print("Snickersit:");
  //  Serial.print(kulutetutSnickersit); Serial.print(" ");
    Serial.print("etäisyys:");
    Serial.print(filteredDistance); Serial.print(" ");
    Serial.print("etäisyys:");
    Serial.print(distance); Serial.print(" ");
  //
  //  Serial.print("ax:");
  //  Serial.print(measuredData.axCalibrated); Serial.print(" ");
  //  Serial.print("ay:");
  //  Serial.print(measuredData.ayCalibrated); Serial.print(" ");
  //  Serial.print("az:");
  //  Serial.print(measuredData.azCalibrated); Serial.print(" ");
  //  Serial.print("gx:");
  //  Serial.print(measuredData.gxCalibrated); Serial.print(" ");
  //  Serial.print("gy:");
  //  Serial.print(measuredData.gyCalibrated); Serial.print(" ");
  //  Serial.print("gz:");
  //  Serial.print(measuredData.gzCalibrated); Serial.print(" ");
//  Serial.print("temp:");
//  Serial.print(measuredData.tempCalibrated); Serial.print(" ");

  //Serial.print(" "); // Printataan kuvaajalle ylä- ja alaraja -20 - 20, jotta kuvaaja ei zoomaile itsestään.
  //Serial.print(-20);
  //Serial.print(" ");
  //Serial.println(80);
  Serial.println();
}

void printSerialMonitor() {
  //Serial.print(aika);
  //  Serial.print(";");
  //Serial.print(distance);
  //  Serial.print(";");
  //  Serial.print(ax);
  //  Serial.print(";");
  //  Serial.print(ay);
  //  Serial.print(";");
  //  Serial.print(az);
  //  Serial.print(";");
  //  Serial.print(axyz);
  //  Serial.println();

}

void printCalibrationData() {
  //  Serial.print(smoothedCalibrationVar);
  //  Serial.println();
}

// Keskeytys vie tänne
//void interruptFunction()
//{
//  interruptCounter++;
//
//  if (deltaDistance >= 0 && smoothedMuutosnopeus > 5) // Jos muutos on positiivinen, niin painoa nostetaan
//  {
//    energialaskuri += PUNTIN_MASSA * PAINOVOIMA * (deltaDistance / 100.0); // Lasketaan ylöspäin nousevan liikkeen kuluttama energia
//  }
//  if (deltaDistance < 0 && smoothedMuutosnopeus < -5) // Jos muutos on negatiivinen, niin painoa lasketaan
//  {
//    energialaskuri += PUNTIN_MASSA * PAINOVOIMA * (deltaDistance / 100.0 * -1) * ALASLASKUN_TYOKERROIN; // Lasketaan alaspäin laskevan liikkeen kuluttama energia
//  }
//
//  // Ehto joka toteutuu joka kymmenes kerta kun keskeytys ajetaan
//  if (interruptCounter >= 10)
//  {
//    interruptCounter = 0;
//    deltaDistance = smoothedDistance - prevDistance; // Lasketaan etäisyyden muutos
//    prevDistance = smoothedDistance; // Tallennetaan vanha etäisyys muistiin ennenkuin luetaan uusi arvo
//    muutosnopeus = (deltaDistance) / (naytevali_ts / (1000.0 / 10.0));
//    smoothedMuutosnopeus = smoothedMuutosnopeus * 0.80 + muutosnopeus * 0.20;
//  }
//}
