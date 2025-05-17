#pragma once
#include <vex.h>

using namespace vex;

class vantadrive
{
    motor_group &left;
    motor_group &right;
    gps &GPS;
    double targetAngle = 0.0;
    const double kP = 2.0;

public:
    DETECTION_OBJECT findTarget(int type);
    vantadrive(motor_group &l, motor_group &r, gps &gps);
    void drive(double targetSpeed);
    void pointTowards(double targetX, double targetY);
    void pointReverse(double targetX, double targetY);
    void goTo(double targetX, double targetY, bool reversed);
    void goTo(int type, bool reversed);
    void turnToTarget();
    void setSpeeds(double leftSpeed, double rightSpeed);
    void stopDrive();
    double distFrom(double targetX, double targetY);
    void turn(double turnAngle);
};