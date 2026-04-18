#include "Motor.h"

Motor::Motor(int pin1, int pin2, int pwmPin, int channel, Position position) : _pin1(pin1), _pin2(pin2), _pwmPin(pwmPin), channel(channel), rel_speed(0), position(position) {}

void Motor::init() {
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  ledcSetup(channel, 20000, 10);
  ledcAttachPin(_pwmPin, channel);
}

void Motor::applySpeed(double rel_speed) {
  this->rel_speed = rel_speed;
  int pwm_value = (int)(1023 * abs(rel_speed));
  if (pwm_value > 1023) pwm_value = 1023;

  if (rel_speed > 0) {
    if (position == Left) {
      digitalWrite(_pin1, LOW);
      digitalWrite(_pin2, HIGH);
    } else {
      digitalWrite(_pin1, HIGH);
      digitalWrite(_pin2, LOW);
    }
  } else if (rel_speed < 0) {
    if (position == Left) {
      digitalWrite(_pin1, HIGH);
      digitalWrite(_pin2, LOW);
    } else {
      digitalWrite(_pin1, LOW);
      digitalWrite(_pin2, HIGH);
    }
  } else {
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
  }
  
  ledcWrite(channel, pwm_value);
}