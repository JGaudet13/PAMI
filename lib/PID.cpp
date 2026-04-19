#include "PID.h"

PID::PID(double kp_val, double ki_val, double kd_val) 
  : kp(kp_val), ki(ki_val), kd(kd_val), prev_error(0), integral(0), prev_time(0) {
}

void PID::reset() {
  prev_error = 0;
  integral = 0;
  prev_time = millis();
}

double PID::calculate(double error) {
  unsigned long current_time = millis();
  double dt = (current_time - prev_time) / 1000.0;  // Convertir en secondes
  
  // Éviter division par zéro
  if (dt == 0) dt = 0.001;
  
  // Terme proportionnel
  double p_term = kp * error;
  
  // Terme intégral avec anti-windup
  integral += error * dt;
  // Limiter l'intégrale pour éviter le windup (-1 à 1)
  if (integral > 1.0) integral = 1.0;
  if (integral < -1.0) integral = -1.0;
  double i_term = ki * integral;
  
  // Terme dérivé
  double derivative = (error - prev_error) / dt;
  double d_term = kd * derivative;
  
  // Sortie PID
  double output = p_term + i_term + d_term;
  
  // Limiter la sortie entre -1 et 1
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;
  
  // Mettre à jour les valeurs précédentes
  prev_error = error;
  prev_time = current_time;
  
  return output;
}

void PID::setGains(double kp_val, double ki_val, double kd_val) {
  kp = kp_val;
  ki = ki_val;
  kd = kd_val;
}
