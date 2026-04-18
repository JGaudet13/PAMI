#include <Arduino.h>
#include "LineSensor.h"

LineSensor::LineSensor(double alpha): alpha(alpha) {}
void LineSensor::init() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){35, 36, 37, 38, 39, 40, 41, 42}, 8);
  qtr.setEmitterPin(21); // Example emitter pin
}

void LineSensor::calibrateLineSensor() {
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(100);
  }
}
double LineSensor::readSensorCOM() {
  uint16_t sensorReadings[8];
  qtr.emittersOn();
  qtr.readCalibrated(sensorReadings);
  
  double mass = 0;
  double moment = 0;
  
  for (int i = 1; i < 7; i++) {
    if(sensorReadings[i] < 500) {
      sensorReadings[i] = 20.0;
    }
    moment += (sensorReadings[i]*(double)positions[i]);
    mass += (sensorReadings[i]);
  }
  qtr.emittersOff();
  double new_value = moment/mass;
  filtered = alpha*new_value + filtered*(1-alpha);
  return filtered;
}

double LineSensor::readSensorBAD() {
    uint16_t sensorReadings[8];
    int max1_idx = 0;
    int max2_idx = 1;
    qtr.readCalibrated(sensorReadings);
    // 1. Trouver les deux indices avec les plus grandes valeurs
    // On suppose que "plus grand" = plus proche de la ligne noire
    for (int i = 1; i < 7; i++) {
        if (sensorReadings[i] > sensorReadings[max1_idx]) {
            max2_idx = max1_idx;
            max1_idx = i;
        } else if (sensorReadings[i] > sensorReadings[max2_idx]) {
            max2_idx = i;
        }
    }

    // Sécurité : si les deux sont trop faibles, on est hors ligne
    if (sensorReadings[max1_idx] < 400) return 0; 

    // 2. Calculer le poids relatif (interpolation linéaire)
    // val1 est la valeur la plus forte, val2 la deuxième
    double val1 = (double)sensorReadings[max1_idx];
    double val2 = (double)sensorReadings[max2_idx];
    
    double p1 = positions[max1_idx];
    double p2 = positions[max2_idx];

    // Formule du centre de masse appliquée à seulement deux points
    // Cela donne une position située entre p1 et p2, proportionnelle à leur force
    double position = (p1 * val1 + p2 * val2) / (val1 + val2);

    
  qtr.emittersOff();
  return position;
}