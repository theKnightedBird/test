#pragma once
#include <vex.h>
#include "pid_controller.h"

using namespace vex;

class vantadrive
{
    motor_group &left;
    motor_group &right;
    gps &GPS;
    AI_RECORD local_map;
    double targetHeading = 0.0;
    pid_controller driveController = pid_controller(1.0, 0.0, 0.1);
    pid_controller turnController = pid_controller(2.0, 0.0, 0.1);
    pid_controller holdController = pid_controller(0.5, 0.0, 0.1);

public:
    DETECTION_OBJECT findTarget(int type);
    vantadrive(motor_group &l, motor_group &r, gps &gps);
    void calibrate();
    void drive(double targetSpeed);
    void turnTo(double targetX, double targetY);
    void turnAwayFrom(double targetX, double targetY);
    bool inAngleTolerance();
    double bearingTo(double targetX, double targetY);
    void driveTo(double targetX, double targetY);
    void driveTo(OBJECT type);
    void turnToTarget();
    void setSpeeds(double leftSpeed, double rightSpeed);
    void stopDrive();
    double distanceTo(double targetX, double targetY);
    void turn(double turnAngle);
    void reverseInto(double targetX, double targetY);
    void reverseInto(OBJECT type);
};