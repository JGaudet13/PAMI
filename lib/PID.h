#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  private:
    double kp;           // Gain proportionnel
    double ki;           // Gain intégral
    double kd;           // Gain dérivé
    double prev_error;   // Erreur précédente
    double integral;     // Somme intégrale
    unsigned long prev_time;  // Temps précédent
    
  public:
    PID(double kp_val, double ki_val, double kd_val);
    
    // Initialiser le PID
    void reset();
    
    // Calculer la sortie du PID
    double calculate(double error);
    
    // Mettre à jour les gains
    void setGains(double kp_val, double ki_val, double kd_val);
};

#endif
