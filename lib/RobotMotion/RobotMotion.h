#pragma once
#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <Arduino.h>
#include "Motor.h"
#include "QE_Manager.h"

class RobotMotion {
  public:
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

  private:
    Motor* _left;
    Motor* _right;
    QE_Manager* _encoder;
    double _wheelDiameterMm;
    double _baseWidthMm;
    int _ticksPerRev;
    double _minSpeed;
    double _maxSpeed;
    double _calibrateFactor;
    void setMotorSpeeds(double leftSpeed, double rightSpeed);
    int calculateCountsForDistanceCm(double distanceCm) const;
    int calculateCountsForAngleDeg(double angleDeg) const;
    void waitForTargetCounts(int targetCountsRight, int targetCountsLeft);
};

#endif