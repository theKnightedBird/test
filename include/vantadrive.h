#pragma once
#include <vex.h>
#include "pid_controller.h"
#include "util_functions.h"

using namespace vex;

class vantadrive
{
    motor_group &left;
    motor_group &right;
    gps &GPS;
    inertial &imu;
    thread periodicThread;
    AI_RECORD local_map;
    double targetHeading = 0.0;
    pid_controller driveController = pid_controller(0.035, 0.0, 0.0);
    pid_controller turnController = pid_controller(0.4, 0.0, 0.0);
    pid_controller holdController = pid_controller(0.3, 0.0, 0.0);

public:
    vantadrive(vex::motor_group &l, vex::motor_group &r, vex::gps &gps, vex::inertial &imu);

    void periodic();
    static void _startPeriodic(void *obj);

    void calibrate();

    DETECTION_OBJECT findTarget(int type);

    void setSpeeds(double moveSpeed, double turnSpeed);
    void stopDrive();

    double distanceTo(double targetX, double targetY);
    double bearingTo(double targetX, double targetY);

    void turnTo(double targetAngle, bool reverse = false);
    void turnTo(double targetX, double targetY, bool reverse = false);
    void turnFor(double angle);
    void spinForTime(double power, double time);

    void driveTo(double targetX, double targetY, bool reverse = false, double tolerance = 75, bool doSecondPass = true);
    void driveTo(OBJECT type, bool reverse = false, double tolerance = 75, bool doSecondPass = true);
    void drive(double power, double dist, bool reverse = false);
};