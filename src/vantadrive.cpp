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

DETECTION_OBJECT vantadrive::find_optimal_target(int type)
{
    DETECTION_OBJECT target = DETECTION_OBJECT();
    jetson_comms.get_data(&local_map);
    double lowest_score = 20000;
    double score;
    // Iterate through detected objects to find the best target of the specified type
    for (DETECTION_OBJECT game_piece : local_map.detections)
    {
        double x = GPS.xPosition();
        double y = GPS.yPosition();
        double x_p = game_piece.mapLocation.x * 1000;
        double y_p = game_piece.mapLocation.y * 1000;

        // set score to the distance
        score = distanceTo(x_p, y_p);
        // don't pick up game pieces that aren't the game peice you want
        if (game_piece.classID != type)
            continue;
        // don't pick up anything too far away, just turn and look for a better one instead.
        if (distanceTo(x_p, y_p) > 5000)
            continue;
        // don't pick up anything outside the borders
        if (fabs(x_p) > 1800 || fabs(y_p) > 1800)
            continue;
        // penalize game pieces close to the border
        if (fabs(x_p) > 1600 || fabs(y_p) > 1600)
            score *= 2;
        // disallow game pieces close to the poles
        if (distance_from_line_segment(
                vec2(x, y),
                vec2(x_p, y_p),
                vec2(0, 600)) < 200)
        {
            continue;
        }
        if (distance_from_line_segment(
                vec2(x, y),
                vec2(x_p, y_p),
                vec2(600, 0)) < 200)
        {
            continue;
        }
        if (distance_from_line_segment(
                vec2(x, y),
                vec2(x_p, y_p),
                vec2(0, -600)) < 200)
        {
            continue;
        }
        if (distance_from_line_segment(
                vec2(x, y),
                vec2(x_p, y_p),
                vec2(-600, 0)) < 200)
        {
            continue;
        }
        // don't go for things in the middle
        if (fabs(x_p) < 600 && fabs(y_p) < 600)
        {
            continue;
        }

        if (score < lowest_score)
        {
            lowest_score = score;
            target = game_piece;
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
    return distance_between(vec2(GPS.xPosition(), GPS.yPosition()), vec2(targetX, targetY));
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
    turnController.reset();
    stopDrive();
    wait(200, msec);
    DETECTION_OBJECT target = find_optimal_target(type);
    while (target.mapLocation.x == 0.0 && target.mapLocation.y == 0.0)
    {
        turnFor(60);
        wait(200, msec);
        target = find_optimal_target(type);
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
