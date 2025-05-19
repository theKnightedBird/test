#include "vantabot.h"

using namespace vex;

vantabot::vantabot(
    vantadrive &d,
    intaker &i,
    digital_out &c) : drive(d), intake(i), clamper(c) {}

void vantabot::grabGoal()
{
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
    clamper.set(false);
    drive.reverseFor(100);
    drive.driveFor(100);
}
