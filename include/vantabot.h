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

    bool hasGoal();                             // a simple binary check on whether we have a goal or not.
    void grab_goal();                           // grabs a mobile goal
    void find_and_score_ring(OBJECT ring_type); // intakes a ring onto a mobile goal
    void score_in_positive_corner();            // puts the goal into the positive corner
    void tipOverGoal();                         // knocks over the goal so nobody can have it
    void go_to_sector();                        // goes to the middle of the closest quadrant, made so GPS is stable when starting a new thing.
};