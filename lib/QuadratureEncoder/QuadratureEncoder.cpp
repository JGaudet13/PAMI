#include "QuadratureEncoder.h"

QuadratureEncoder::QuadratureEncoder(int pinA, int pinB, pcnt_unit_t unit) 
    : _pinA(pinA), _pinB(pinB), _unit(unit) {}

void QuadratureEncoder::begin() {
    pcnt_config_t pcnt_config = {};
    
    // 1. Assign Pins
    pcnt_config.pulse_gpio_num = _pinA;
    pcnt_config.ctrl_gpio_num = _pinB;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = _unit;
    
    pcnt_config.pos_mode = PCNT_COUNT_INC;  // Count on rising edge
    pcnt_config.neg_mode = PCNT_COUNT_DIS;  // Ignore falling edge
    
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; 
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    
    
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;


    pcnt_unit_config(&pcnt_config);
    

    pcnt_set_filter_value(_unit, 1000);
    pcnt_filter_enable(_unit);

    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);
}

int16_t QuadratureEncoder::getCount() {
    int16_t count = 0;
    pcnt_get_counter_value(_unit, &count);
    return count;
}

void QuadratureEncoder::reset() {
    pcnt_counter_clear(_unit);
}