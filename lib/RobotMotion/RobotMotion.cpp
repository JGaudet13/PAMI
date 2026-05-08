#include "RobotMotion.h"

RobotMotion::RobotMotion(Motor* leftMotor, Motor* rightMotor, QE_Manager* encoderManager,
                         double wheelDiameterMm, double baseWidthMm, int ticksPerRev)
    : _left(leftMotor), _right(rightMotor), _encoder(encoderManager),
      _wheelDiameterMm(wheelDiameterMm), _baseWidthMm(baseWidthMm),
      _ticksPerRev(ticksPerRev), _minSpeed(0.0), _maxSpeed(1.0) {}

void RobotMotion::init() {
  stop();
  setSpeedLimits(0.0, 1.0);
}

void RobotMotion::stop() {
  if (_left) _left->applySpeed(0.0);
  if (_right) _right->applySpeed(0.0);
}

void RobotMotion::setSpeedLimits(double minSpeed, double maxSpeed) {
  _minSpeed = minSpeed;
  _maxSpeed = maxSpeed;
}

void RobotMotion::setMotorSpeeds(double leftSpeed, double rightSpeed) {
  if (leftSpeed > _maxSpeed) leftSpeed = _maxSpeed;
  if (leftSpeed < -_maxSpeed) leftSpeed = -_maxSpeed;
  if (rightSpeed > _maxSpeed) rightSpeed = _maxSpeed;
  if (rightSpeed < -_maxSpeed) rightSpeed = -_maxSpeed;

  if (_left) _left->applySpeed(leftSpeed);
  if (_right) _right->applySpeed(rightSpeed);
}

int RobotMotion::calculateCountsForDistanceCm(double distanceCm) const {
  double circumferenceCm = (3.14159265358979323846 * _wheelDiameterMm) / 10.0;
  double revolutions = distanceCm / circumferenceCm;
  return (int)round(revolutions * _ticksPerRev);
}

int RobotMotion::calculateCountsForAngleDeg(double angleDeg) const {
  double turnCircumferenceMm = 3.14159265358979323846 * _baseWidthMm;
  double arcDistanceMm = (turnCircumferenceMm * angleDeg) / 360.0;
  double circumferenceMm = 3.14159265358979323846 * _wheelDiameterMm;
  double revolutions = arcDistanceMm / circumferenceMm;
  return (int)round(revolutions * _ticksPerRev);
}

void RobotMotion::waitForTargetCounts(int targetCountsRight, int targetCountsLeft) {
  if (!_encoder) return;

  const int startRight = _encoder->right_enc.getCount();
  const int startLeft = _encoder->left_enc.getCount();

  while (true) {
    int currentRight = _encoder->right_enc.getCount();
    int currentLeft = _encoder->left_enc.getCount();
    int deltaRight = abs(currentRight - startRight);
    int deltaLeft = abs(currentLeft - startLeft);

    bool rightDone = deltaRight >= abs(targetCountsRight);
    bool leftDone = deltaLeft >= abs(targetCountsLeft);

    if (rightDone && leftDone) break;
    delay(2);
  }

  stop();
}

void RobotMotion::moveForwardCm(double distanceCm, double speed) {
  if (!_encoder) return;
  if (distanceCm <= 0) return;

  int targetCounts = calculateCountsForDistanceCm(distanceCm);
  _encoder->right_enc.reset();
  _encoder->left_enc.reset();

  setMotorSpeeds(speed, speed);
  waitForTargetCounts(targetCounts, targetCounts);
}

void RobotMotion::moveBackwardCm(double distanceCm, double speed) {
  if (!_encoder) return;
  if (distanceCm <= 0) return;

  int targetCounts = calculateCountsForDistanceCm(distanceCm);
  _encoder->right_enc.reset();
  _encoder->left_enc.reset();

  setMotorSpeeds(-speed, -speed);
  waitForTargetCounts(targetCounts, targetCounts);
}

void RobotMotion::turnRightDeg(double angleDeg, double speed) {
  if (!_encoder) return;
  if (angleDeg <= 0) return;

  int targetCounts = calculateCountsForAngleDeg(angleDeg);
  _encoder->right_enc.reset();
  _encoder->left_enc.reset();

  setMotorSpeeds(speed, -speed);
  waitForTargetCounts(targetCounts, targetCounts);
}

void RobotMotion::turnLeftDeg(double angleDeg, double speed) {
  if (!_encoder) return;
  if (angleDeg <= 0) return;

  int targetCounts = calculateCountsForAngleDeg(angleDeg);
  _encoder->right_enc.reset();
  _encoder->left_enc.reset();

  setMotorSpeeds(-speed, speed);
  waitForTargetCounts(targetCounts, targetCounts);
}

void RobotMotion::avoidObstacle( double distanceCm, double speed) {
  Serial.println(" Évitement obstacle...");

  turnRightDeg(90, 0.6);
  delay(100);
  moveForwardCm(20, speed);
  delay(100);
  turnLeftDeg(90, 0.6);
  delay(100);
  moveForwardCm(40, speed);
  delay(100);
  turnLeftDeg(90, 0.6);
  delay(100);
  moveForwardCm(20, speed);
  turnRightDeg(90, 0.6);
  delay(400);
}