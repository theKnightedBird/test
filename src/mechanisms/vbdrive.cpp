#include "vbdrive.hpp"
using namespace vex;

VB_Drive::VB_Drive(
    motor_group left,
    motor_group right,
    double gps_port,
    double wheelTravel,
    double trackWidth,
    double wheelBase,
    distanceUnits unit,
    double externalGearRatio) : drive(left, right, GPS, wheelTravel, trackWidth, wheelBase, unit, externalGearRatio),
                                GPS(gps_port)
{
}
double VB_Drive::distanceTo(double target_x, double target_y)
{
    double distance = sqrt(pow((target_x - GPS.xPosition(vex::distanceUnits::cm)), 2) + pow((target_y - GPS.yPosition(vex::distanceUnits::cm)), 2));
    return distance;
}
double VB_Drive::calculateBearing(double currX, double currY, double targetX, double targetY)
{
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0)
    {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0)
    {
        bearing_deg += 360;
    }

    return bearing_deg;
}
void VB_Drive::turnTo(double angle, int tolerance, int speed)
{

    double current_heading = GPS.heading();
    double angle_to_turn = angle - current_heading;

    // Normalize the angle to the range [-180, 180]
    while (angle_to_turn > 180)
        angle_to_turn -= 360;
    while (angle_to_turn < -180)
        angle_to_turn += 360;

    // Determine the direction to turn (left or right)
    turnType direction = angle_to_turn > 0 ? turnType::left : turnType::right;
    drive.turn(direction, speed, velocityUnits::pct);
    while (1)
    {

        current_heading = GPS.heading();
        // Check if the current heading is within a tolerance of degrees to the target
        if (current_heading > (angle - tolerance) && current_heading < (angle + tolerance))
        {
            break;
        }
    }
    drive.stop();
}

void VB_Drive::driveFor(int heading, double distance, int speed)
{
    // Determine the smallest degree of turn
    double angle_to_turn = heading - GPS.heading();
    while (angle_to_turn > 180)
        angle_to_turn -= 360;
    while (angle_to_turn < -180)
        angle_to_turn += 360;

    // Decide whether to move forward or backward
    // Allos for a 5 degree margin of error that defaults to forward
    directionType direction = fwd;
    if (std::abs(angle_to_turn) > 105)
    {
        angle_to_turn += angle_to_turn > 0 ? -180 : 180;
        direction = directionType::rev;
    }
    else if (std::abs(angle_to_turn) < 75)
    {
        angle_to_turn += angle_to_turn > 0 ? 180 : -180;
        direction = directionType::fwd;
    }

    drive.driveFor(direction, distance, vex::distanceUnits::cm, speed, velocityUnits::pct);
}
void VB_Drive::moveToPosition(double target_x, double target_y, double target_theta)
{
    // Calculate the angle to turn to face the target
    double intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), target_x, target_y);
    // Turn to face the target
    turnTo(intialHeading, 10, 15);
    double distance = distanceTo(target_x, target_y);
    // Move to the target, only 30% of total distance to account for error
    driveFor(intialHeading, distance * 0.3, 50);

    // Recalculate the heading and distance to the target
    double heading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), target_x, target_y);
    turnTo(heading, 15, 10);
    distance = distanceTo(target_x, target_y);
    // Move to the target, completing the remaining distance
    driveFor(heading, distance, 20);

    // Turn to the final target heading if specified, otherwise use current heading
    if (target_theta == -1)
    {
        target_theta = GPS.heading();
    }
    turnTo(target_theta, 5, 2);
}
DETECTION_OBJECT VB_Drive::findTarget(int type)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
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

void VB_Drive::goToObject(OBJECT type)
{
    DETECTION_OBJECT target = findTarget(type);
    // If no target found, turn and try to find again
    if (target.mapLocation.x == 0 && target.mapLocation.y == 0)
    {
        drive.turnFor(45, rotationUnits::deg, 50, velocityUnits::pct);
        target = findTarget(0);
    }
    // Move to the detected target's position
    moveToPosition(target.mapLocation.x * 100, target.mapLocation.y * 100);
}
void VB_Drive::calibrate()
{
    GPS.calibrate();
    waitUntil(!GPS.isCalibrating());
}