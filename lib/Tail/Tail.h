#pragma once
#include <Arduino.h>
#include "Servo.h"
class Tail {
public:
  int pin;
  int minPulseWidth;
  int maxPulseWidth;
  int max_angle;
  int min_angle;
  int channel;
  CustomServo servo_instance;

  Tail();
  void attach(int pin = 14, int minPulseWidth = 544, int maxPulseWidth = 2400, int frequency = 50, int channel = 3, int resolution = 12);
  void detach();
  void write(int angle);
  void setPeriodHertz(int frequency);

  void Wag();
  void initialPosition();
  void setAngleLimits(int min_angle = 70, int max_angle = 150);
};