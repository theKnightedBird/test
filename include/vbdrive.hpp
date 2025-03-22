#pragma once
#include <vex.h>

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
        double wheelTravel,
        double trackWidth,
        double wheelBase,
        distanceUnits unit,
        double externalGearRatio);
    void calibrate();
    double distanceTo(double target_x, double target_y);
    double calculateBearing(double currX, double currY, double targetX, double targetY);
    void turnTo(double angle, int tolerance, int speed);
    void driveFor(int heading, double distance, int speed);
    void moveToPosition(double target_x, double target_y, double target_theta = -1);
    DETECTION_OBJECT findTarget(int type);
    void goToObject(OBJECT type);

private:
    smartdrive drive;
    gps GPS;
};