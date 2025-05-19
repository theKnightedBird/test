#pragma once
#include <vex.h>
#include "vantadrive.h"

using namespace vex;

class vantabot
{
    digital_out &clamp;
    motor_group &intake;
    bool hasGoal;
public:
    vantadrive &drive;
    bool isRed;
    vantabot(vantadrive &d, digital_out &c, motor_group &i, bool red);
    void grabGoal();
    void findAndScoreRing();
    void guardCorner();
    void clearCorner();
    void scoreNeutralWallStake();
    void hang();
    void scoreAlliance();
    void putGoalInCorner();
    void stealOtherCorner();
    void knockOverGoal();
};