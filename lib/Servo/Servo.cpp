#include "Servo.h"
#include <Arduino.h>
CustomServo::CustomServo(): freq(50), channel(0), resolution(12), pin(-1), angle(0) {}

void CustomServo::attach(int pin, int min, int max, int freq, int channel, int resolution) {
  this->pin = pin;
  this->min = min;
  this->max = max;
  this->freq = freq;
  this->channel = channel;
  this->resolution = resolution;
  pinMode(pin, OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
}

void CustomServo::detach() {
  this->pin = -1;
}

void CustomServo::write(int angle) {
  if(pin == -1) return; // Not attached
  this->angle = angle;
  float pulseWidth = (float)this->min + (float)angle/180.0 * (this->max - this->min); //(microseconds) Map angle (0-180) to pulse width (500-2500 microseconds)
  float duty = (float)pulseWidth / (1E6 / (float)freq); // Convert pulse width to duty cycle
  int bit_duty = duty * ((1 << resolution) - 1); // Scale duty cycle to resolution
  ledcWrite(channel, bit_duty);
}