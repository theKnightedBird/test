#include "robot.h"

using namespace vex;

vantabot::vantabot(
    vantadrive &d,
    digital_out &c,
    motor_group &i,
    bool red) : drive(d), clamp(c), intake(i), isRed(red) {}

void vantabot::grabGoal()
{
    drive.driveTo(MobileGoal, true);
    clamp.set(true);
    
}

void vantabot::findAndScoreRing()
{
    int ringType = RedRing ? BlueRing : isRed;
    intake.spin(forward);
    drive.driveTo(ringType);
}
