#pragma once
#include <vex.h>
#include "vantadrive.h"
#include "intaker.h"

using namespace vex;

class vantabot
{
    vantadrive &drive;
    intaker &intake;
    digital_out &clamper;
    distance &clamp_sensor;
    bool holdingGoal = false;

public:
    vantabot(vantadrive &d, intaker &i, digital_out &c, distance &cs);

    bool hasGoal();
    void grabGoal();
    void findAndScoreRing(OBJECT ring_type);
    void scoreInPositiveCorner();
    void tipOverGoal();
};