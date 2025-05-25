#include "vantadrive.h"
using namespace vex;

vantadrive::vantadrive(motor_group &l, motor_group &r, gps &gps, inertial &i) : left(l), right(r), GPS(gps), imu(i), periodicThread(_startPeriodic, this)
{
    targetHeading = GPS.heading();
}

void vantadrive::periodic()
{
    double lastHeading = 0.0;
    double lastRotation = 0.0;
    while (true)
    {
        // Update imu with gps
        // if (GPS.heading() != lastHeading)
        //     imu.setHeading(GPS.heading(), deg);
        // if (GPS.rotation() != lastRotation)
        //     imu.setRotation(GPS.rotation(), deg);
        lastHeading = GPS.heading();
        lastRotation = GPS.rotation();
        wait(20, msec);
    }
}

void vantadrive::_startPeriodic(void *obj)
{
    static_cast<vantadrive *>(obj)->periodic();
}

void vantadrive::calibrate()
{
    GPS.calibrate();
    imu.calibrate();
    waitUntil(!GPS.isCalibrating() && !imu.isCalibrating());
    wait(250, msec);
    imu.setHeading(GPS.heading(), deg);
    imu.setRotation(GPS.rotation(), deg);
    targetHeading = imu.heading();
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
    double desaturateFator = fmax(1.0, fmax(fabs(leftSpeed) / 100.0, fabs(rightSpeed) / 100.0));
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

void vantadrive::turnTo(double targetAngle, bool reverse)
{
    targetHeading = targetAngle;
    if (reverse)
        targetHeading += 180;
    turnController.reset();
    double prev_angle = imu.heading();
    double turn_speed;
    while (fabs(angle_between(targetHeading, imu.heading())) > 2.0 || fabs(imu.heading() - prev_angle) > 0.5)
    {
        turn_speed = turnController.calculate(angle_between(targetHeading, imu.heading()));
        setSpeeds(0.0, turn_speed);
        prev_angle = imu.heading();
        wait(20, msec);
    }
    stopDrive();
}

void vantadrive::turnTo(double targetX, double targetY, bool reverse)
{
    double dx = targetX - GPS.xPosition();
    double dy = targetY - GPS.yPosition();
    double angle = atan2(dy, dx) * 180 / M_PI;
    angle = fmod(450 - angle, 360);
    turnTo(angle, reverse);
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

void vantadrive::driveTo(double targetX, double targetY, bool reverse, double tolerance, bool doSecondPass)
{
    double drive_speed;
    double turn_speed;

    // first pass
    turnTo(targetX, targetY, reverse);
    driveController.reset();
    holdController.reset();
    while (distanceTo(targetX, targetY) > 5 * tolerance)
    {
        drive_speed = driveController.calculate(distanceTo(targetX, targetY));
        if (reverse)
            drive_speed *= -1;
        targetHeading = bearingTo(targetX, targetY);
        if (reverse)
            targetHeading += 180;
        turn_speed = holdController.calculate(angle_between(targetHeading, imu.heading()));
        setSpeeds(
            drive_speed,
            turn_speed);
        wait(20, msec);
    }
    stopDrive();

    // second pass
    if (doSecondPass)
    {
        turnTo(targetX, targetY, reverse);
        driveController.reset();
        holdController.reset();
        while (distanceTo(targetX, targetY) > tolerance)
        {
            drive_speed = driveController.calculate(distanceTo(targetX, targetY));
            if (reverse)
                drive_speed *= -1;
            targetHeading = bearingTo(targetX, targetY);
            if (reverse)
                targetHeading += 180;
            turn_speed = holdController.calculate(angle_between(targetHeading, imu.heading()));
            setSpeeds(
                drive_speed,
                turn_speed);
            wait(20, msec);
        }
        stopDrive();
    }
}

void vantadrive::driveTo(OBJECT type, bool reverse, double tolerance, bool doSecondPass)
{
    DETECTION_OBJECT target = findTarget(type);
    while (target.mapLocation.x == 0 && target.mapLocation.y == 0)
    {
        turnFor(10);
        target = findTarget(type);
    }
    driveTo(target.mapLocation.x * 1000, target.mapLocation.y * 1000, reverse, tolerance, doSecondPass);
}

void vantadrive::drive(double power, double distance, bool reverse)
{
    holdController.reset();
    targetHeading = imu.heading();
    double startX = GPS.xPosition();
    double startY = GPS.yPosition();
    double drive_speed;
    double turn_speed;
    while (distanceTo(startX, startY) < distance)
    {
        drive_speed = reverse ? -power : power;
        turn_speed = holdController.calculate(angle_between(targetHeading, imu.heading()));
        setSpeeds(
            drive_speed,
            turn_speed);
        wait(20, msec);
    }
    stopDrive();
}
