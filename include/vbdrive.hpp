#pragma once
#include "vex.h"

using namespace vex;

enum OBJECT
{
    MobileGoal,
    RedRing,
    BlueRing,
    BothRings
};

class VB_Drive
{
public:
    VB_Drive(
        motor_group left,
        motor_group right,
        double gps_port,
        double wheelTravel = 320,
        double trackWidth = 320,
        double wheelBase = 130,
        distanceUnits unit = distanceUnits::mm,
        double externalGearRatio = 1.0);
    void calibrate();
    double distanceTo(double target_x, double target_y);
    double calculateBearing(double currX, double currY, double targetX, double targetY);
    void turnTo(double angle, int tolerance, int speed);
    void driveFor(int heading, double distance, int speed);
    void moveToPosition(double target_x, double target_y, double target_theta = -1);
    DETECTION_OBJECT findTarget(int type);
    void goToObject(OBJECT type);
    gps GPS;
    smartdrive drive;

private:
};