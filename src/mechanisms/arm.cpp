#include "arm.hpp"

using namespace vex;

Arm::Arm(int port)
{
    motor motor_1 = motor(port);
    arm_motor = motor_group(motor_1);
    state = IDLE;
}

Arm::Arm(int port_1, int port_2)
{
    motor motor_1 = motor(port_1);
    motor motor_2 = motor(port_2);
    arm_motor = motor_group(motor_1, motor_2);
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