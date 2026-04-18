#include "QE_Manager.h"
#include <cmath>

// Constructor initializes encoders with specific hardware units


QE_Manager::QE_Manager(int rA, int rB, int lA, int lB, double base, double diam) 
    : right_enc(rA, rB, PCNT_UNIT_0), 
      left_enc(lA, lB, PCNT_UNIT_1), 
      baseLength(base), 
      wheelDiam(diam) {}

void QE_Manager::init() {
    right_enc.begin();
    left_enc.begin();
    resetDistanceCounter();
}

void QE_Manager::update() {
    // 1. Get raw angular velocities (rad/s) from the encoders
    double raw_right = right_enc.getAngV();
    double raw_left = left_enc.getAngV();

    // 2. Low-pass filter to smooth out quantization noise (Alpha filter)
    // Adjust 0.8 to be higher for more responsiveness, lower for more smoothness
    double alpha = 0.7; 
    right_speed = (alpha * raw_right) + (1.0 - alpha) * right_speed;
    left_speed = (alpha * raw_left) + (1.0 - alpha) * left_speed;

    // 3. Integration for Distance Tracking
    if (updateDistance) {
        static uint64_t lastMicros = 0;
        uint64_t now = esp_timer_get_time();
        double dt = (now - lastMicros) / 1000000.0;
        
        if (lastMicros != 0) { // Skip the very first tick
            distanceRight += (right_speed * (wheelDiam / 2.0)) * dt;
            distanceLeft += (left_speed * (wheelDiam / 2.0)) * dt;
        }
        lastMicros = now;
    }
}

// Kinematics: The "Robot-Level" speeds
double QE_Manager::getRotSpeed() {
    // Angular velocity of the robot chassis (rad/s)
    // Omega = (Vr - Vl) / L
    double vr = right_speed * (wheelDiam / 2.0);
    double vl = left_speed * (wheelDiam / 2.0);
    return (vr - vl) / baseLength;
}

double QE_Manager::getTotalSpeed() {
    // Linear velocity of the robot center (m/s)
    // V_center = (Vr + Vl) / 2
    double vr = right_speed * (wheelDiam / 2.0);
    double vl = left_speed * (wheelDiam / 2.0);
    return (vr + vl) / 2.0;
}

// Getters for individual wheel speeds
double QE_Manager::getRightSpeed() { return -1*right_speed; }
double QE_Manager::getLeftSpeed() { return -1*left_speed; }

// Distance Counter Management
void QE_Manager::startDistanceCounter() { updateDistance = true; }

void QE_Manager::resetDistanceCounter() {
    distanceRight = 0;
    distanceLeft = 0;
}

double QE_Manager::getDistanceRight() { return distanceRight; }
double QE_Manager::getDistanceLeft() { return distanceLeft; }
