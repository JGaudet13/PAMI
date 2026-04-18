#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h" // The correct header for your version

class QuadratureEncoder {
private:
    int _pinA;
    int _pinB;
    int resolution = 360;
    double _ticksToRad;
    pcnt_unit_t _unit;
    int16_t _lastCount;
    uint64_t _lastTime;
    double _velocity;
public:
    QuadratureEncoder(int pinA, int pinB, pcnt_unit_t unit);
    void begin();
    int16_t getCount();
    double getAngV();
    void reset();
};

#endif