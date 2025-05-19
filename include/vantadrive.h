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
    vantadrive(vex::motor_group &l, vex::motor_group &r, vex::gps &gps);

    void calibrate();

    DETECTION_OBJECT findTarget(int type);

    void setSpeeds(double moveSpeed, double turnSpeed);
    void stopDrive();

    double distanceTo(double targetX, double targetY);
    double bearingTo(double targetX, double targetY);

    void turnTo(double targetAngle);
    void turnTo(double targetX, double targetY);
    void turnAwayFrom(double targetX, double targetY);
    void turnFor(double angle);
    void spinForTime(double power, double time);

    void driveTo(double targetX, double targetY);
    void driveTo(OBJECT type);
    void driveFor(double dist);

    void reverseInto(double targetX, double targetY);
    void reverseInto(OBJECT type);
    void reverseFor(double dist);
};