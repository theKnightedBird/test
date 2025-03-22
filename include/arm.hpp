#pragma once
#include "vex.h"

using namespace vex;

enum arm_state
{
    IDLE,
    GRABBING,
    SCORING
};

class Arm
{
public:
    Arm(int port);
    Arm(int port1, int port2);
    void idle();
    void grab();
    void score();
    void periodic();

private:
    motor_group arm_motor;
    arm_state state;
    thread t;
};
