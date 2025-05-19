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
    drive.reverseInto(MobileGoal);
    clamper.set(true);
}

void vantabot::findAndScoreRing()
{
    intake.intake();
    drive.driveTo(allianceRing);
    wait(3, sec);
    intake.stop();
}

void vantabot::scoreInPositiveCorner()
{
    drive.driveTo(allianceRing == RedRing ? 1500 : -1500, -1500);
    drive.turnTo(allianceRing == RedRing ? 45 : 135);
    clamper.set(false);
    drive.reverseFor(100);
    drive.driveFor(100);
    intake.resetCount();
}
