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
    bool hasGoal;

public:
    vantabot(vantadrive &d, intaker &i, digital_out &c);
    void grabGoal();
    void findAndScoreRing();
    void scoreInPositiveCorner();
};