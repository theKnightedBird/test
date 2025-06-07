#include "vantabot.h"

using namespace vex;

vantabot::vantabot(
    vantadrive &d,
    intaker &i,
    digital_out &c,
    vex::distance &cs) : drive(d), intake(i), clamper(c), clamp_sensor(cs) {}

bool vantabot::hasGoal()
{
    return clamp_sensor.isObjectDetected();
}

void vantabot::grab_goal()
{
    go_to_sector();
    drive.driveTo(MobileGoal, true, 50.0, false);
    clamper.set(false);
    drive.drive(33, 300, true);
    wait(200, msec);
    clamper.set(true);
    wait(200, msec);
}

void vantabot::find_and_score_ring(OBJECT ring_type)
{
    go_to_sector();
    intake.intake();
    drive.driveTo(ring_type);
    drive.drive(50, 100);
    drive.stopDrive();
    wait(3, sec);
    intake.stop();
}

void vantabot::score_in_positive_corner()
{
    vec dest = vec({(allianceRing == RedRing) ? 1300.0 : -1300.0,
                    -1300.0});
    vec dest2 = vec(
        {(allianceRing == RedRing) ? 1800.0 : -1800.0,
         -1800});

    go_to_sector();

    drive.driveTo(dest[0], dest[1], true);
    drive.turnTo(dest2[0], dest2[1], true);
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
    vec position = vec({drive.GPS.xPosition(), drive.GPS.yPosition()});
    vec sectors[] = {
        vec({800.0, 800.0}),
        vec({800.0, -800.0}),
        vec({-800.0, 800.0}),
        vec({-800.0, -800.0})};
    vec closest;
    double closestDist = 1000000000;
    for (vec sector : sectors)
    {
        if (vec::dist_between(position, sector) < closestDist)
        {
            closestDist = vec::dist_between(position, sector);
            closest = vec(sector);
        }
    }
    drive.driveTo(closest[0], closest[1], false);
}
