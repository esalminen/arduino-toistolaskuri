/*

  Tällä ohjelmalla demotaan kiihtyvyys-anturin toimintaa ja kiihtyvyys-näkymiä Serial Plotteria hyödyntäen
  8.5.2017 -Jaakko Kaski-

  Muutettu sopivaksi GY-521 I2C väylää käyttävälle anturille. 1.2.2022 / ESa

*/
#include <Wire.h>
#include <MsTimer2.h>
#include "AccSensor.h"
#include "SonarSensor.h"

AccSensor *accSensor;
AccSensorMeasureData measuredData;
SonarSensor *sonarSensor;

const int MPU = 0x68; // GY-521 väyläosoite

// Muuttujamäärittelyt. Huomaa, että desimaalierotin on piste!
unsigned long aika = 0; // Aikaleima (ms), tyyppinä "pitkä, merkitön" muoto, koska INT-tyyppisenä numeroavaruus tulee n. puolessa minuutissa täyteen.

float ax = 0.0;  // x-kanavan kiihtyvyysarvo SI-muodossa (m/s^2)
float ay = 0.0;
float az = 0.0;
float axyz = 0.0;
float axAvg = 0.0;
float ayAvg = 0.0;
float azAvg = 0.0;
float axyzAvg = 0.0;
float axPlot = 0.0;
float ayPlot = 0.0;
float azPlot = 0.0;
float vx = 0.0;
float vy = 0.0;
float vz = 0.0;
float vxyz = 0.0;
float vxAvg = 0.0;
float vyAvg = 0.0;
float vzAvg = 0.0;
float vxPlot = 0.0;
float vyPlot = 0.0;
float vzPlot = 0.0;
float vxyzPlot = 0.0;
float vFilterRad = 5.0;
float aFilterRad = 5.0;
float vxyzFilterRad = 30.0;
unsigned long prevTime = 0;
float smoothedCalibrationVar = 0.0;
int SisaanTunniste = 0;

// Toistolaskurin muuttujat
const int YLARAJA = 40; // cm
const int ALARAJA = 20; // cm
const int PUNTIN_MASSA = 500; // Nostettavan painon massa [kg]
const float PAINOVOIMA = 9.81; // m/s^2
const float ALASLASKUN_TYOKERROIN = 0.6; // Potentiaalienergian muutos alaslaskulla lasketaan kulutetuksi energiaksi kertoimella 0.6
const float SNICKERS_ENERGIA = 481.0 ; // Patukan energia [kcal]
const float KEHON_HYOTYSUHDE = 0.15; // Keho kuluttaa energiaa työtä tehdessä 15 % hyötysuhteella

bool ylhaalla = false; // Kun tila on false, niin tila on alhaalla
int toistomaara = 0; // Edestakaisten toistojen määrä
int minDistance = YLARAJA; // Nostoliikkeen minimietäisyys
int maxDistance = ALARAJA; // Nostoliikkeen maksimietäisyys
int naytevali_ts = 10; //Näyteväli(ms), säädä sopivaksi, 2ms taitaa olla minimi keskeytyksellä.
int interruptCounter = 0; // Laskuri pitempiä keskeytysintervalleja varten
float energialaskuri = 0; // Kulutettu energia jouleina [J]
float kulutettuEnergiaKcal = 0; // Kulutettu energia kilokaloreina [kcal]
float kulutetutSnickersit = 0; // Kulutetut Snickers-patukat
float muutosnopeus = 0.0; // Etäisyyden muutosnopeus cm/s
float smoothedMuutosnopeus = 0.0; // Etäisyyden muutosnopeus cm/s filtteröitynä
float prevDistance = 0.0; // Edellisen ohjelmakierron etäisyys
float deltaDistance = 0.0; // Edellisen ohjelmakierron ja nykyisen ohjelmakierron erotus


// Keskeytys vie tänne
void flash()
{
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
}

void setup() {

  //  ax = 0.0006 * acX - 0.4275; //Kalibrointiyhtälö tehty 16.2.2022 / ESa
  //  ay = 0.0006 * acY + 0.0390;
  //  az = 0.0006 * acZ - 0.2623;

  accSensor = new AccSensor(MPU);
  accSensor->init(4, 0, 0);
  accSensor->calibrateAcc(0.0006, -0.4275, 0.0006, 0.0390, 0.0006, -0.2623);//

  int gndPin = 11;
  int echoPin = 10;
  int trigPin = 9;
  int vccPin = 8;
  float maxDistance = 100.0;
  float minDistance = 0.0;

  sonarSensor = new SonarSensor(gndPin, echoPin, trigPin, vccPin, maxDistance, minDistance);

  Serial.begin (19200); // Tämä täytyy valita myös Serial Monitorista samaksi
  while (Serial.available() != 0)
  {
    // Odotellaan että yhteys käynnistyy jos tässä sattuu olemaan viivettä. 0 tarkoittaa että yhteys on.
  }

  // Asetetaan laukaisuintervalli ja ajettava funktio. Sitten käynnistys
  //MsTimer2::set(naytevali_ts, flash); //Ajastimen alustus; näyteväli annetaan millisekunteina määrittelyissä.
  //MsTimer2::start();

}

void loop() {
  
  // eka sisäänmenolla annetaan 1ms aikaa käynnistyä. Muuten 1. arvo on pelkkää häiriötä.
  if (SisaanTunniste == 0)
  {
    delay(1); // 1ms viive käynnistymiselle
    SisaanTunniste = 1; // muutetaan testattava muuttuja jotta tänne ei enää tulla
  }

  accSensor->measure();
  measuredData = accSensor->getMeasurements();



  // Aikaleima (ms)
  prevTime = aika;
  aika = millis(); // Aikaleima luetaan ennen laskentaa, tosi SERIAL PLOTTER EI KÄYTÄ TÄTÄ!
  float Ts = (aika - prevTime) / 1000.0;

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

  // Toisto- ja energialaskurin toimintalogiikka

  // Ylärajan yläpuolella laitetaan ylhäällä-tila aktiiviseksi, ja alarajan alapuolella deaktivoidaan.
//  if (!ylhaalla && distance >= YLARAJA) {
//    toistomaara++; // Lisätään toistojen määrää yhdellä
//    maxDistance = YLARAJA; // Asetetaan maksimietäisyyden lähtöarvoksi ylaraja kun siirrytään ylhäällä tilaan.
//    ylhaalla = true;
//  }
//
//  if (ylhaalla && distance <= ALARAJA) {
//    minDistance = ALARAJA; // Asetetaan minimietäisyyden lähtöarvoksi alaraja kun siirrytään alhaalla tilaan.
//    ylhaalla = false;
//  }
//
//  // Tallenna maksimiarvoa etäisyydestä kun ollaan ylhaalla
//  if (ylhaalla && distance > maxDistance) maxDistance = distance;
//
//  // Tallenna minimiarvoa etäisyydestä kun ollaan alhaalla (ylhaalla tila deaktiivinen)
//  if (!ylhaalla && distance < minDistance) minDistance = distance;

  // Lasketaan monta snickersiä on palanut
  kulutettuEnergiaKcal = energialaskuri / 4.1868 / 1000.0;
  kulutetutSnickersit = (kulutettuEnergiaKcal / KEHON_HYOTYSUHDE) / SNICKERS_ENERGIA;



  printSerialPlotter();
  //printSerialMonitor();
  //printCalibrationData();

  //Viive ennen seuraavaa kierrosta niin dataa tulee kohtuullisella tahdilla
  // Jos mittaa hirmuisen nopeasti niin numerinen derivaatta näyttää kohinaisemmalta
  // vaikka siinä toki olisi tietoa enemmän!
  delay(2);
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
  //  Serial.print("toistomäärä[kpl]:");
  //  Serial.print(toistomaara); Serial.print(" ");
  //  Serial.print("muutosnopeus_ds/dt:");
  //  Serial.print(smoothedMuutosnopeus); Serial.print(" ");
  //Serial.print("energialaskuri[J]:");
  //Serial.print(energialaskuri); Serial.print(" ");
  //  Serial.print("energia[kcal]:");
  //  Serial.print(kulutettuEnergiaKcal); Serial.print(" ");
  //  Serial.print("Snickersit:");
  //  Serial.print(kulutetutSnickersit); Serial.print(" ");
  //  Serial.print("etäisyys:");
  //  Serial.print(distance); Serial.print(" ");
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
  Serial.print("temp:");
  Serial.print(measuredData.tempCalibrated); Serial.print(" ");

  //Serial.print(" "); // Printataan kuvaajalle ylä- ja alaraja -20 - 20, jotta kuvaaja ei zoomaile itsestään.
  //Serial.print(-20);
  //Serial.print(" ");
  //Serial.println(80);
  Serial.println();
}

void printSerialMonitor() {
  Serial.print(aika);
  Serial.print(";");
  //Serial.print(distance);
  Serial.print(";");
  Serial.print(ax);
  Serial.print(";");
  Serial.print(ay);
  Serial.print(";");
  Serial.print(az);
  //  Serial.print(";");
  //  Serial.print(axyz);
  Serial.println();

}

void printCalibrationData() {
  Serial.print(smoothedCalibrationVar);
  Serial.println();
}
