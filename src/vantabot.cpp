#include "vantabot.h"

using namespace vex;

vantabot::vantabot(
    vantadrive &d,
    intaker &i,
    digital_out &c,
    distance &cs) : drive(d), intake(i), clamper(c), clamp_sensor(cs) {}

bool vantabot::hasGoal()
{
    return clamp_sensor.objectDistance(mm) < 100;
}

void vantabot::grabGoal()
{
    go_to_sector();
    clamper.set(false);
    drive.driveTo(MobileGoal, true, 50.0, false);
    drive.drive(33, 300, true);
    clamper.set(true);
    go_to_sector();
    while (!clamp_sensor.isObjectDetected())
    {
        clamper.set(false);
        drive.driveTo(MobileGoal, true, 50.0, false);
        drive.drive(33, 300, true);
        clamper.set(true);
        go_to_sector();
    }
}

void vantabot::findAndScoreRing(OBJECT ring_type)
{
    go_to_sector();
    intake.intake();
    drive.driveTo(ring_type);
    drive.drive(50, 100);
    drive.stopDrive();
    wait(3, sec);
    intake.stop();
}

void vantabot::scoreInPositiveCorner()
{
    vec2 dest = vec2(
        allianceRing == RedRing ? 1300 : -1300,
        -1300);
    vec2 dest2 = vec2(
        allianceRing == RedRing ? 1800 : -1800,
        -1800);

    go_to_sector();

    drive.driveTo(dest.x, dest.y, true);
    drive.turnTo(dest2.x, dest2.y, true);
    clamper.set(false);
    drive.drive(50, 150, true);
    drive.drive(50, 150);
    intake.resetCount();
}

void vantabot::tipOverGoal()
{
    drive.spinForTime(100, 2);
    clamper.set(false);
    drive.stopDrive();
    intake.resetCount();
}

void vantabot::go_to_sector()
{
    vec2 position = vec2(drive.GPS.xPosition(), drive.GPS.yPosition());
    vec2 sectors[] = {
        vec2(800, 800),
        vec2(800, -800),
        vec2(-800, 800),
        vec2(-800, -800)};
    vec2 closest;
    double closestDist = 1000000000;
    for (vec2 sector : sectors)
    {
        if (distance_between(position, sector) < closestDist)
        {
            closestDist = distance_between(position, sector);
            closest = sector;
        }
    }
    drive.driveTo(closest.x, closest.y, false, 200.0);
}
