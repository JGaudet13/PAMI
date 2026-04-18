#include <Arduino.h>

#include "Tail.h"

#include "US_Manager.h"
#include "QE_Manager.h"
#include "Motor.h"
#include "LineSensor.h"


Motor m_right(7, 15, 16, 1, Right);
Motor m_left(6, 5, 4, 0, Left);
QE_Manager QE(18,8, 10, 9, 78.0, 35.0);
Tail tail;
LineSensor lineSensor(0.2);
double alpha = 0.7;
void setup() {
  Serial.begin(9600);
  m_right.init();
  m_left.init();
  tail.attach(14);
  tail.setAngleLimits(60, 120);
  tail.initialPosition();
  lineSensor.init();
  QE.init();
}

void loop() { 
  lineSensor.calibrateLineSensor();
  m_left.applySpeed(1);
  while (true) {
    QE.update();
    Serial.print(">SpeedR:");
    Serial.println(QE.getRightSpeed());
    Serial.print(">SpeedL:");
    Serial.println(QE.getLeftSpeed());
    delay(10);
  }
}

