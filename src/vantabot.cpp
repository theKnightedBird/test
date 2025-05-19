#include "vantabot.h"

using namespace vex;

vantabot::vantabot(
    vantadrive &d,
    digital_out &c,
    motor_group &i,
    optical &o,
    bool red) : drive(d), clamp(c), intake(i), intakeSensor(o), isRed(red) {}

void vantabot::grabGoal()
{
    drive.reverseInto(MobileGoal);
    clamp.set(true);
}

void vantabot::findAndScoreRing()
{
    OBJECT ringType = isRed ? RedRing : BlueRing;
    intake.spin(forward);
    drive.driveTo(ringType);

}

void vantabot::scoreInPositiveCorner()
{
    drive.driveTo(isRed ? 1500 : -1500, -1500);
    clamp.set(false);
    drive.reverseFor(100);
    drive.driveFor(100);
}
