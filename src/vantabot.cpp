#include "vantabot.h"

using namespace vex;

vantabot::vantabot(
    vantadrive &d,
    intaker &i,
    digital_out &c,
    distance &cs) : drive(d), intake(i), clamper(c), clamp_sensor(cs) {}

bool vantabot::hasGoal()
{
    return clamp_sensor.isObjectDetected();
}

void vantabot::grabGoal()
{
    clamper.set(false);
    drive.driveTo(MobileGoal, true, 50.0, false);
    drive.drive(33, 300, true);
    clamper.set(true);
}

void vantabot::findAndScoreRing(OBJECT ring_type)
{
    intake.intake();
    drive.driveTo(ring_type);
    drive.drive(50, 300);
    wait(3, sec);
    intake.stop();
}

void vantabot::scoreInPositiveCorner()
{
    drive.driveTo(allianceRing == RedRing ? 1500 : -1500, -1500);
    drive.turnTo(allianceRing == RedRing ? 45 : 135);
    clamper.set(false);
    drive.drive(50, 300, true);
    drive.drive(50, 300);
    intake.resetCount();
}

void vantabot::tipOverGoal()
{
    drive.spinForTime(100, 2);
    clamper.set(false);
    drive.stopDrive();
    intake.resetCount();
}
