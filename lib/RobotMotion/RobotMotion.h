#pragma once
#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <Arduino.h>
#include "Motor.h"
#include "QE_Manager.h"

enum ObstacleBehavior {
  OBSTACLE_IGNORE,
  OBSTACLE_AVOIDANCE,
  OBSTACLE_GAME
};

class RobotMotion {
  public:
    typedef double (*ObstacleDistanceReader)();

    RobotMotion(Motor* leftMotor, Motor* rightMotor, QE_Manager* encoderManager,
                double wheelDiameterMm = 22, double baseWidthMm = 85, int ticksPerRev = 441);
    double calibrateFactor;
    void init();
    void stop();

    void moveForwardCm(double distanceCm, double speed);
    void moveBackwardCm(double distanceCm, double speed);
    void turnRightDeg(double angleDeg, double speed);
    void turnLeftDeg(double angleDeg, double speed);
    

    void avoidObstacle(double distanceCm, double speed );

    void setSpeedLimits(double minSpeed, double maxSpeed);
    void setMotorTrim(double leftTrim, double rightTrim);
    void setMovementTimeoutMs(unsigned long timeoutMs);
    void setObstacleReader(ObstacleDistanceReader reader);
    void setObstacleBehavior(ObstacleBehavior behavior, double thresholdCm = 20.0,
                             unsigned long gamePauseMs = 3000,
                             unsigned long gameCooldownMs = 2000);

  private:
    Motor* _left;
    Motor* _right;
    QE_Manager* _encoder;
    double _wheelDiameterMm;
    double _baseWidthMm;
    int _ticksPerRev;
    double _minSpeed;
    double _maxSpeed;
    double _leftTrim;
    double _rightTrim;
    double _activeLeftSpeed;
    double _activeRightSpeed;
    unsigned long _movementTimeoutMs;
    ObstacleDistanceReader _obstacleReader;
    ObstacleBehavior _obstacleBehavior;
    double _obstacleThresholdCm;
    unsigned long _gameObstaclePauseMs;
    unsigned long _gameObstacleCooldownMs;
    unsigned long _lastGameObstaclePauseEndMs;
    bool _gameObstacleArmed;
    double _calibrateFactor;
    void setMotorSpeeds(double leftSpeed, double rightSpeed);
    int calculateCountsForDistanceCm(double distanceCm) const;
    int calculateCountsForAngleDeg(double angleDeg) const;
    bool obstacleDetected() const;
    unsigned long handleObstacleIfNeeded(double resumeLeftSpeed, double resumeRightSpeed);
    void waitForTargetCounts(int targetCountsRight, int targetCountsLeft);
};

#endif
