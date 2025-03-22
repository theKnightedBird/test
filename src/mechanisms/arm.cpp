#include "arm.hpp"

using namespace vex;

Arm::Arm(int port)
{
    arm_motor(motor(port));
    state = IDLE;
}

Arm::Arm(int port1, int port2)
{
    arm_motor(motor(port1), motor(port2));
    state = IDLE;
}

void Arm::idle()
{
    state = IDLE;
}

void Arm::grab()
{
    state = GRABBING;
}

void Arm::score()
{
    state = SCORING;
}

void Arm::periodic()
{
    switch (state)
    {
    case IDLE:
        arm_motor.stop();
    case GRABBING:
        arm_motor.spinToPosition(150.0, degrees, false);
        break;
    case SCORING:
        arm_motor.spinToPosition(210.0, degrees, false);
        break;
    default:
        break;
    }
}