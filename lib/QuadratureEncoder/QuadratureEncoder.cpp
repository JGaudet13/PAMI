#include "QuadratureEncoder.h"

QuadratureEncoder::QuadratureEncoder(int pinA, int pinB, pcnt_unit_t unit) 
    : _pinA(pinA), _pinB(pinB), _unit(unit) {
        pinMode(_pinA, INPUT_PULLUP);
        pinMode(_pinB, INPUT_PULLUP);
    }

void QuadratureEncoder::begin() {
    _ticksToRad = 2.0*3.1415/(double)resolution;
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
    _lastTime = esp_timer_get_time();
}

int16_t QuadratureEncoder::getCount() {
    int16_t count = 0;
    pcnt_get_counter_value(_unit, &count);
    return count;
}
double QuadratureEncoder::getAngV() {
    int16_t currentCount = getCount();
    uint64_t currentTime = esp_timer_get_time();
    
    int16_t diff = currentCount - _lastCount;
    double dt = (currentTime - _lastTime) / 1000000.0;

    // 1. Prevent NaN from high-speed calls
    if (dt <= 0.000001) return _velocity; 

    // 2. Logic: If pulses happened, calculate new speed
    if (diff != 0) {
        _velocity = (diff * _ticksToRad) / dt;
        _lastCount = currentCount; // Only update these when a pulse occurs
        _lastTime = currentTime;
    } 
    else {
        // 3. The "Decay" Logic: If no pulses, is it actually stopped?
        // If the time since the last pulse is longer than it SHOULD take 
        // to see a single pulse at the current speed, we are slowing down/stopped.
        if (dt > 0.02) { // 20ms threshold is usually safe for Eurobot
            _velocity = 0;
            _lastTime = currentTime; // Keep dt moving so we don't divide by 0 later
        }
    }

    return _velocity;
}

void QuadratureEncoder::reset() {
    pcnt_counter_clear(_unit);
}

