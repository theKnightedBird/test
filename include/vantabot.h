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
    vex::distance &clamp_sensor;
    bool holdingGoal = false;

public:
    vantabot(vantadrive &d, intaker &i, digital_out &c, vex::distance &cs);

    bool hasGoal();
    void grab_goal();
    void find_and_score_ring(OBJECT ring_type);
    void score_in_positive_corner();
    void tipOverGoal();
    void go_to_sector();
};