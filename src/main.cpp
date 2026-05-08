#include <Arduino.h>

#include "Tail.h"
#include "Motor.h"

#include "SensingThread.h"
#include "PIDLineFollower.h"
#include "RobotMotion.h"
Motor m_left(15, 7, 16, 1, Right);
Motor m_right(5, 6, 4, 0, Left);
Tail tail;
SensingThread sensors;
PIDLineFollower lf(&m_right, &m_left, &sensors);
RobotMotion motion(&m_right, &m_left, &sensors.encoder);
PID pid(0.1, 0, 0);
double alpha = 0.6;
void setup() {
  Serial.begin(115200);
  m_right.init();
  m_left.init();
  tail.attach(14);
  tail.setAngleLimits(60, 120);
  tail.initialPosition();  
  sensors.setup_sensors();
  sensors.start_sensing_thread();
  

  motion.setSpeedLimits(-1.0, 1.0);
  motion.moveForwardCm(100, 0.5);
  motion.turnRightDeg(55, 0.5);
  motion.moveForwardCm(20, .8);
  
}

void loop() { 
tail.Wag();
}
