#include "PIDLineFollower.h"

// Constructeur
PIDLineFollower::PIDLineFollower(Motor* right_motor, Motor* left_motor, SensingThread* sensor_thread)
  : motor_right(right_motor), motor_left(left_motor), sensors(sensor_thread),
    base_speed(0.5), max_speed(1.0), min_speed(-1.0), initialized(false) {
  
  // Créer les instances PID
  pid_line_follower = new PID(.008, 0 , 0.004);  // Gains par défaut pour suivi de ligne
  pid_motor_right = new PID(.1, 0, 0);    // Gains par défaut pour moteur droit
  pid_motor_left = new PID(.1, 0, 0);     // Gains par défaut pour moteur gauche
}

// Destructeur
PIDLineFollower::~PIDLineFollower() {
  delete pid_line_follower;
  delete pid_motor_right;
  delete pid_motor_left;
}

// Initialisation
void PIDLineFollower::init() {
  if (pid_line_follower && pid_motor_right && pid_motor_left) {
    pid_line_follower->reset();
    pid_motor_right->reset();
    pid_motor_left->reset();
    initialized = true;
  }
}

// Configuration des gains PID du suivi de ligne
void PIDLineFollower::setLineFollowerGains(double kp, double ki, double kd) {
  if (pid_line_follower) {
    pid_line_follower->setGains(kp, ki, kd);
  }
}

// Configuration des gains PID des moteurs
void PIDLineFollower::setMotorGains(double kp, double ki, double kd) {
  if (pid_motor_right && pid_motor_left) {
    pid_motor_right->setGains(kp, ki, kd);
    pid_motor_left->setGains(kp, ki, kd);
  }
}

// Configuration de la vitesse de base
void PIDLineFollower::setBaseSpeed(double speed) {
  base_speed = constrain(speed, 0.0, 1.0);
}

// Configuration des limites de vitesse
void PIDLineFollower::setSpeedLimits(double min, double max) {
  min_speed = constrain(min, -1.0, 0.0);
  max_speed = constrain(max, 0.0, 1.0);
}

// Méthode principale de suivi de ligne
void PIDLineFollower::followLine() {
  if (!initialized) return;
  
  sensorInfoPID info = sensors->sensor_values();
  double correction = pid_line_follower->calculate(info.LS_error);
  
  // Générer les vitesses cibles
  double target_right = base_speed - correction;
  double target_left = base_speed + correction;
  
  // Appliquer les contraintes de vitesse
  target_right = constrainSpeed(target_right);
  target_left = constrainSpeed(target_left);
  
  // ===== ÉTAPE 3: APPLIQUER AUX MOTEURS =====
  updateMotors(target_right, target_left);
}

// Arrêter les moteurs
void PIDLineFollower::stop() {
  updateMotors(0.0, 0.0);
}

// Réinitialiser les PID
void PIDLineFollower::reset() {
  if (pid_line_follower) pid_line_follower->reset();
  if (pid_motor_right) pid_motor_right->reset();
  if (pid_motor_left) pid_motor_left->reset();
}

// Getters pour debug
double PIDLineFollower::getLineError() {
  sensorInfoPID info = sensors->sensor_values();
  return info.LS_error;
}

double PIDLineFollower::getRightTargetSpeed() {
  double line_correction = pid_line_follower->calculate(getLineError());
  return constrainSpeed(base_speed - line_correction);
}

double PIDLineFollower::getLeftTargetSpeed() {
  double line_correction = pid_line_follower->calculate(getLineError());
  return constrainSpeed(base_speed + line_correction);
}

double PIDLineFollower::getRightActualSpeed() {
  return sensors->encoder.getRightSpeed();
}

double PIDLineFollower::getLeftActualSpeed() {
  return sensors->encoder.getLeftSpeed();
}

double PIDLineFollower::getRightPWM() {
  double line_correction = pid_line_follower->calculate(getLineError());
  return constrainSpeed(base_speed - line_correction);
}

double PIDLineFollower::getLeftPWM() {
  double line_correction = pid_line_follower->calculate(getLineError());
  return constrainSpeed(base_speed + line_correction);
}

// Méthodes privées
void PIDLineFollower::updateMotors(double right_pwm, double left_pwm) {
  if (motor_right && motor_left) {
    motor_right->applySpeed(right_pwm);
    motor_left->applySpeed(left_pwm);
  }
}

double PIDLineFollower::constrainSpeed(double speed) {
  return constrain(speed, min_speed, max_speed);
}