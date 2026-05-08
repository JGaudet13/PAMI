#pragma once
#include <Arduino.h>
enum Position {
  Right, 
  Left
};
class Motor {
  private:
    int _pin1;
    int _pin2;
    int _pwmPin;
    int channel;
    double rel_speed;
    Position position;
  public:
    Motor(int pin1, int pin2, int pwmPin, int channel, Position position);
    void init();
    void applySpeed(double rel_speed); //from -1  to 1, where negative is backward and positive is forward

};