#include <Arduino.h>

#include "Tail.h"
#include "Motor.h"

#include "SensingThread.h"


Motor m_right(7, 15, 16, 1, Right);
Motor m_left(6, 5, 4, 0, Left);

Tail tail;
SensingThread sensors;
double alpha = 0.7;
void setup() {
  Serial.begin(115200);
  m_right.init();
  m_left.init();
  tail.attach(14);
  tail.setAngleLimits(60, 120);
  tail.initialPosition();
  
  sensors.setup_sensors();
  sensors.start_sensing_thread();
}

void loop() { 
  sensorInfoPID info = sensors.sensor_values();
  Serial.print(">ls:");
  Serial.println(info.LS_error);
  Serial.print(">ultrasonic:");
  Serial.println(info.obstacle);
  Serial.print(">right_motor:");
  Serial.println(info.right_speed);
  Serial.print(">left motor:");
  Serial.println(info.left_speed);
  delay(100);
}

