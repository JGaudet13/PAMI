#include <Arduino.h>

#include "Tail.h"
#include "MotorController.h"
#include "US_Manager.h"
#include "QuadratureEncoder.h"
#include "Motor.h"
#include "LineSensor.h"


Motor m_right(7, 15, 16, 1, Right);
Motor m_left(6, 5, 4, 0, Left);
Tail tail;
LineSensor lineSensor;
double alpha = 0.7;
void setup() {
  Serial.begin(9600);
  m_right.init();
  m_left.init();
  tail.attach(14);
  tail.setAngleLimits(60, 120);
  tail.initialPosition();
  lineSensor.init();
  
}

void loop() { 
  lineSensor.calibrateLineSensor();
  double filtered = 0;
  while (true) {
    double pos = lineSensor.readSensorCOM();
    filtered = alpha*pos + (1-alpha)*filtered;
    Serial.print(">Pos:");
    Serial.println(filtered);
    delay(100);
  }
}

