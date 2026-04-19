#ifndef PID_LINE_FOLLOWER_H
#define PID_LINE_FOLLOWER_H

#include <Arduino.h>
#include "PID.h"
#include "Motor.h"
#include "SensingThread.h"

class PIDLineFollower {
  private:
    // PID de suivi de ligne (haut niveau)
    PID* pid_line_follower;
    
    // PID des moteurs (bas niveau)
    PID* pid_motor_right;
    PID* pid_motor_left;
    
    // Références aux moteurs et capteurs
    Motor* motor_right;
    Motor* motor_left;
    SensingThread* sensors;
    
    // Paramètres
    double base_speed;
    double max_speed;
    double min_speed;
    
    // État interne
    bool initialized;
    
  public:
    // Constructeur
    PIDLineFollower(Motor* right_motor, Motor* left_motor, SensingThread* sensor_thread);
    
    // Destructeur
    ~PIDLineFollower();
    
    // Initialisation
    void init();
    
    // Configuration des gains PID
    void setLineFollowerGains(double kp, double ki, double kd);
    void setMotorGains(double kp, double ki, double kd);
    
    // Configuration des vitesses
    void setBaseSpeed(double speed);
    void setSpeedLimits(double min, double max);
    
    // Contrôle principal
    void followLine();
    
    // Méthodes utilitaires
    void stop();
    void reset();
    
    // Getters pour debug
    double getLineError();
    double getRightTargetSpeed();
    double getLeftTargetSpeed();
    double getRightActualSpeed();
    double getLeftActualSpeed();
    double getRightPWM();
    double getLeftPWM();
    
  private:
    // Méthodes internes
    void updateMotors(double right_pwm, double left_pwm);
    double constrainSpeed(double speed);
};

#endif