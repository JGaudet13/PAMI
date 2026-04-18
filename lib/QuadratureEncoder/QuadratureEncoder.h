#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h" // The correct header for your version

class QuadratureEncoder {
private:
    int _pinA;
    int _pinB;
    pcnt_unit_t _unit;

public:
    QuadratureEncoder(int pinA, int pinB, pcnt_unit_t unit);
    void begin();
    int16_t getCount();
    void reset();
};

#endif