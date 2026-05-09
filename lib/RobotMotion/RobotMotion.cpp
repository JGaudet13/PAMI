#include "RobotMotion.h"

RobotMotion::RobotMotion(Motor* leftMotor, Motor* rightMotor, QE_Manager* encoderManager,
                         double wheelDiameterMm, double baseWidthMm, int ticksPerRev)
    : _left(leftMotor), _right(rightMotor), _encoder(encoderManager),
      _wheelDiameterMm(wheelDiameterMm), _baseWidthMm(baseWidthMm),
      _ticksPerRev(ticksPerRev), _minSpeed(0.0), _maxSpeed(1.0),
      _leftTrim(1.0), _rightTrim(1.0), _activeLeftSpeed(0.0), _activeRightSpeed(0.0),
      _movementTimeoutMs(30000),
      _obstacleReader(nullptr),
      _obstacleBehavior(OBSTACLE_IGNORE), _obstacleThresholdCm(20.0),
      _gameObstaclePauseMs(3000), _gameObstacleCooldownMs(2000),
      _lastGameObstaclePauseEndMs(0), _gameObstacleArmed(true) {}

void RobotMotion::init() {
  stop();
  setSpeedLimits(0.0, 1.0);
}

void RobotMotion::stop() {
  _activeLeftSpeed = 0.0;
  _activeRightSpeed = 0.0;
  if (_left) _left->applySpeed(0.0);
  if (_right) _right->applySpeed(0.0);
}

void RobotMotion::setSpeedLimits(double minSpeed, double maxSpeed) {
  _minSpeed = minSpeed;
  _maxSpeed = maxSpeed;
}

void RobotMotion::setMotorTrim(double leftTrim, double rightTrim) {
  _leftTrim = leftTrim;
  _rightTrim = rightTrim;
}

void RobotMotion::setMovementTimeoutMs(unsigned long timeoutMs) {
  _movementTimeoutMs = timeoutMs;
}

void RobotMotion::setObstacleReader(ObstacleDistanceReader reader) {
  _obstacleReader = reader;
}

void RobotMotion::setObstacleBehavior(ObstacleBehavior behavior, double thresholdCm,
                                      unsigned long gamePauseMs,
                                      unsigned long gameCooldownMs) {
  _obstacleBehavior = behavior;
  _obstacleThresholdCm = thresholdCm;
  _gameObstaclePauseMs = gamePauseMs;
  _gameObstacleCooldownMs = gameCooldownMs;
  _lastGameObstaclePauseEndMs = 0;
  _gameObstacleArmed = true;
}

void RobotMotion::setMotorSpeeds(double leftSpeed, double rightSpeed) {
  leftSpeed *= _leftTrim;
  rightSpeed *= _rightTrim;

  if (leftSpeed > _maxSpeed) leftSpeed = _maxSpeed;
  if (leftSpeed < -_maxSpeed) leftSpeed = -_maxSpeed;
  if (rightSpeed > _maxSpeed) rightSpeed = _maxSpeed;
  if (rightSpeed < -_maxSpeed) rightSpeed = -_maxSpeed;

  _activeLeftSpeed = leftSpeed;
  _activeRightSpeed = rightSpeed;

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

bool RobotMotion::obstacleDetected() const {
  if (!_obstacleReader || _obstacleBehavior == OBSTACLE_IGNORE) return false;

  double obstacleCm = _obstacleReader();
  return obstacleCm > 0.0 && obstacleCm <= _obstacleThresholdCm;
}

unsigned long RobotMotion::handleObstacleIfNeeded(double resumeLeftSpeed, double resumeRightSpeed) {
  if (!obstacleDetected()) {
    _gameObstacleArmed = true;
    return 0;
  }

  if (_obstacleBehavior == OBSTACLE_GAME && _lastGameObstaclePauseEndMs > 0 &&
      millis() - _lastGameObstaclePauseEndMs < _gameObstacleCooldownMs) {
    return 0;
  }

  if (_obstacleBehavior == OBSTACLE_GAME && !_gameObstacleArmed) {
    return 0;
  }

  const unsigned long pauseStart = millis();
  stop();

  if (_obstacleBehavior == OBSTACLE_AVOIDANCE) {
    Serial.println("Obstacle detected. Waiting until clear.");
    while (obstacleDetected()) {
      delay(20);
    }
    Serial.println("Obstacle cleared. Resuming path.");
  } else if (_obstacleBehavior == OBSTACLE_GAME && _gameObstacleArmed) {
    Serial.println("Obstacle detected. Pausing for game mode.");
    delay(_gameObstaclePauseMs);
    Serial.println("Game obstacle pause complete. Resuming path.");
    _lastGameObstaclePauseEndMs = millis();
    _gameObstacleArmed = false;
  } else {
    return 0;
  }

  setMotorSpeeds(resumeLeftSpeed, resumeRightSpeed);
  return millis() - pauseStart;
}

void RobotMotion::waitForTargetCounts(int targetCountsRight, int targetCountsLeft) {
  if (!_encoder) return;

  const int startRight = _encoder->right_enc.getCount();
  const int startLeft = _encoder->left_enc.getCount();
  const unsigned long startTime = millis();
  unsigned long pausedTimeMs = 0;

  while (true) {
    pausedTimeMs += handleObstacleIfNeeded(_activeLeftSpeed, _activeRightSpeed);

    int currentRight = _encoder->right_enc.getCount();
    int currentLeft = _encoder->left_enc.getCount();
    int deltaRight = abs(currentRight - startRight);
    int deltaLeft = abs(currentLeft - startLeft);

    bool rightDone = deltaRight >= abs(targetCountsRight);
    bool leftDone = deltaLeft >= abs(targetCountsLeft);

    if (rightDone && leftDone) break;

    if (_movementTimeoutMs > 0 && millis() - startTime - pausedTimeMs >= _movementTimeoutMs) {
      Serial.println("RobotMotion timeout waiting for encoder target.");
      Serial.println("Right delta: " + String(deltaRight) + " / " + String(abs(targetCountsRight)));
      Serial.println("Left delta: " + String(deltaLeft) + " / " + String(abs(targetCountsLeft)));
      break;
    }

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
