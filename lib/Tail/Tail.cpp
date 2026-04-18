#include "Tail.h"

Tail::Tail() : pin(14), minPulseWidth(500), maxPulseWidth(2500) {}

void Tail::attach(int pin, int minPulseWidth, int maxPulseWidth, int frequency, int channel, int resolution) {
  this->pin = pin;
  this->minPulseWidth = minPulseWidth;
  this->maxPulseWidth = maxPulseWidth;
  servo_instance.attach(pin, minPulseWidth, maxPulseWidth, frequency, channel, resolution);

}

void Tail::detach() {
  servo_instance.detach();
}

void Tail::write(int angle) {
  servo_instance.write(angle);
}

void Tail::Wag() {
  for (int pos = min_angle; pos <= max_angle; pos++) {
    servo_instance.write(pos);
    delay(5);
  }
  for (int pos = max_angle; pos >= min_angle; pos--) {
    servo_instance.write(pos);
    delay(5);
  }
}

void Tail::initialPosition() {
  servo_instance.write((min_angle + max_angle) / 2);
}

void Tail::setAngleLimits(int min_angle, int max_angle) {
  this->min_angle = min_angle;
  this->max_angle = max_angle;
}
