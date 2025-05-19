#include "vantadrive.h"
using namespace vex;

vantadrive::vantadrive(motor_group &l, motor_group &r, gps &gps) : left(l), right(r), GPS(gps)
{
}
void vantadrive::calibrate()
{
    GPS.calibrate();
    waitUntil(!GPS.isCalibrating());
    waitUntil(jetson_comms.get_packets() > 0);
};

DETECTION_OBJECT vantadrive::findTarget(int type)
{
    DETECTION_OBJECT target;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    // Iterate through detected objects to find the closest target of the specified type
    for (int i = 0; i < local_map.detectionCount; i++)
    {
        double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && local_map.detections[i].classID == type)
        {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    return target;
}

void vantadrive::setSpeeds(double moveSpeed, double turnSpeed)
{
    double leftSpeed = moveSpeed + turnSpeed;
    double rightSpeed = moveSpeed - turnSpeed;

    // desat the speeds
    double desaturateFator = fmax(100.0, fmax(fabs(leftSpeed), fabs(rightSpeed)));
    leftSpeed /= desaturateFator;
    rightSpeed /= desaturateFator;

    left.spin(fwd, leftSpeed, pct);
    right.spin(fwd, rightSpeed, pct);
}

void vantadrive::stopDrive()
{
    left.stop(brake);
    right.stop(brake);
}

double vantadrive::distanceTo(double targetX, double targetY)
{
    return sqrt(pow(targetX - GPS.xPosition(mm), 2) + pow(targetY - GPS.yPosition(mm), 2));
}

double vantadrive::bearingTo(double targetX, double targetY)
{
    double dx = targetX - GPS.xPosition();
    double dy = targetY - GPS.yPosition();
    double angle = atan2(dy, dx) * 180 / M_PI;
    angle = fmod(450 - angle, 360);
    return angle;
}

void vantadrive::turnTo(double targetAngle)
{
    targetHeading = targetAngle;
    turnController.reset();
    double angle_error = fmod(targetHeading - GPS.heading() + 180, 360) - 180;
    while (angle_error > 1.0)
    {
        angle_error = fmod(targetHeading - GPS.heading() + 180, 360) - 180;
        setSpeeds(0.0, turnController.calculate(angle_error));
        wait(5, msec);
    }
    stopDrive();
}

void vantadrive::turnTo(double targetX, double targetY)
{
    double dx = targetX - GPS.xPosition();
    double dy = targetY - GPS.yPosition();
    double angle = atan2(dy, dx) * 180 / M_PI;
    angle = fmod(450 - angle, 360);
    turnTo(angle);
}

void vantadrive::turnAwayFrom(double targetX, double targetY)
{
    double dx = targetX - GPS.xPosition();
    double dy = targetY - GPS.yPosition();
    double angle = atan2(dy, dx) * 180 / M_PI;
    angle = fmod(450 - angle, 360);
    turnTo(angle + 180);
}

void vantadrive::turnFor(double angle)
{
    turnTo(targetHeading + angle);
}

void vantadrive::spinForTime(double power, double time)
{
    double start_time = timer::system();
    while (timer::system() - start_time < time)
    {
        setSpeeds(0.0, power);
    }
}

void vantadrive::driveTo(double targetX, double targetY)
{
    turnTo(targetX, targetY);
    driveController.reset();
    holdController.reset();
    double angle_error;
    while (distanceTo(targetX, targetY) > 100)
    {
        targetHeading = bearingTo(targetX, targetY);
        angle_error = fmod(targetHeading - GPS.heading() + 180, 360) - 180;
        setSpeeds(
            driveController.calculate(distanceTo(targetX, targetY)),
            holdController.calculate(angle_error));
        wait(5, msec);
    }
    turnTo(targetX, targetY);
    driveController.reset();
    holdController.reset();
    double angle_error;
    while (distanceTo(targetX, targetY) > 10)
    {
        targetHeading = bearingTo(targetX, targetY);
        angle_error = fmod(targetHeading - GPS.heading() + 180, 360) - 180;
        setSpeeds(
            driveController.calculate(distanceTo(targetX, targetY)),
            holdController.calculate(angle_error));
        wait(5, msec);
    }
    stopDrive();
}

void vantadrive::driveTo(OBJECT type)
{
    DETECTION_OBJECT target = findTarget(type);
    while (target.mapLocation.x == 0 && target.mapLocation.y == 0)
    {
        turnFor(10);
        wait(50, msec);
        target = findTarget(type);
    }
    driveTo(target.mapLocation.x * 1000, target.mapLocation.y * 1000);
}

void vantadrive::driveFor(double dist)
{
    driveTo(100 * cos(targetHeading), 100 * sin(targetHeading));
}

void vantadrive::reverseInto(double targetX, double targetY)
{
    turnAwayFrom(targetX, targetY);
    driveController.reset();
    holdController.reset();
    double angle_error;
    while (distanceTo(targetX, targetY) > 100)
    {
        targetHeading = bearingTo(targetX, targetY) + 180;
        angle_error = fmod(targetHeading - GPS.heading() + 180, 360) - 180;
        setSpeeds(
            driveController.calculate(distanceTo(targetX, targetY)),
            holdController.calculate(angle_error));
        wait(5, msec);
    }
    turnAwayFrom(targetX, targetY);
    driveController.reset();
    holdController.reset();
    double angle_error;
    while (distanceTo(targetX, targetY) > 10)
    {
        targetHeading = bearingTo(targetX, targetY) + 180;
        angle_error = fmod(targetHeading - GPS.heading() + 180, 360) - 180;
        setSpeeds(
            driveController.calculate(distanceTo(targetX, targetY)),
            holdController.calculate(angle_error));
        wait(5, msec);
    }
    stopDrive();
}
void vantadrive::reverseInto(OBJECT type)
{
    DETECTION_OBJECT target = findTarget(type);
    while (target.mapLocation.x == 0 && target.mapLocation.y == 0)
    {
        turnFor(10);
        wait(50, msec);
        target = findTarget(type);
    }
    reverseInto(target.mapLocation.x * 1000, target.mapLocation.y * 1000);
}
void vantadrive::reverseFor(double dist)
{
    reverseInto(-100 * cos(targetHeading), -100 * sin(targetHeading));
}
