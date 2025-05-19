#pragma once
#include <vex.h>
#include "vantadrive.h"

using namespace vex;

class vantabot
{
    vantadrive &drive;
    digital_out &clamp;
    motor_group &intake;
    optical &intakeSensor;
    bool hasGoal;

public:
    bool isRed;
    vantabot(vantadrive &d, digital_out &c, motor_group &i, optical &o, bool red);
    void grabGoal();
    void findAndScoreRing();
    void scoreInPositiveCorner();
};