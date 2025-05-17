#include "vantadrive.h"
using namespace vex;

vantadrive::vantadrive(motor_group &l, motor_group &r, gps &gps) : left(l), right(r), GPS(gps)
{
    GPS.calibrate();
};

void vantadrive::setSpeeds(double leftSpeed, double rightSpeed)
{
    left.setVelocity(leftSpeed, rpm);
    right.setVelocity(rightSpeed, rpm);
}

void vantadrive::stopDrive()
{
    left.stop();
    right.stop();
}

DETECTION_OBJECT vantadrive::findTarget(int type)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    // Iterate through detected objects to find the closest target of the specified type
    for (int i = 0; i < local_map.detectionCount; i++)
    {
        double distance = distFrom(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && local_map.detections[i].classID == type)
        {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    return target;
}


double vantadrive::distFrom(double targetX, double targetY)
{
    return sqrt(pow(targetX - GPS.xPosition(), 2) + pow(targetY - GPS.yPosition(), 2));
}

void vantadrive::turnToTarget()
{
    double angleToTurn;
    do
    {
        angleToTurn = targetAngle - GPS.heading();
        angleToTurn = fmod(angleToTurn + 180, 360) - 180;
        setSpeeds(kP * angleToTurn, -kP * angleToTurn);
    } while (abs(angleToTurn) > 0.5);
    left.stop();
    right.stop();
}

void vantadrive::turn(double targetAngle)
{
    this->targetAngle = targetAngle;
    turnToTarget();
}

void vantadrive::drive(double targetSpeed)
{
    turnToTarget();
    double currentAngle = GPS.heading();
    double angleToTurn = targetAngle - currentAngle;
    angleToTurn = fmod(angleToTurn + 180, 360) - 180;
    left.setVelocity(targetSpeed + kP * angleToTurn, rpm);
    right.setVelocity(targetSpeed - kP * angleToTurn, rpm);
}

void vantadrive::pointTowards(double targetX, double targetY)
{
    double dx = targetX - GPS.xPosition();
    double dy = targetY - GPS.yPosition();
    double angle = atan2(dy, dx) * 180 / M_PI;
    angle = fmod(450 - angle, 360);
    turn(angle);
}

void vantadrive::pointReverse(double targetX, double targetY)
{
    double dx = targetX - GPS.xPosition();
    double dy = targetY - GPS.yPosition();
    double angle = atan2(dy, dx) * 180 / M_PI;
    angle = fmod(450 - angle, 360);
    turn(angle + 180);
}

void vantadrive::goTo(double targetX, double targetY, bool reversed = false)
{
    if (reversed)
    {
        pointReverse(targetX, targetY);
        do
        {
            drive(-kP * distFrom(targetX, targetY));
        } while (distFrom(targetX, targetY) > 10);
    }
    else
    {
        pointTowards(targetX, targetY);
        do
        {
            drive(kP * distFrom(targetX, targetY));
        } while (distFrom(targetX, targetY) > 10);
    }
}

void vantadrive::goTo(int type, bool reversed = false)
{
    DETECTION_OBJECT target = findTarget(type);
    while (target.mapLocation.x == 0 && target.mapLocation.y == 0)
    {
        turn(targetAngle + 1);
        target = findTarget(type);
    }
    goTo(target.mapLocation.x, target.mapLocation.y, reversed);
}
