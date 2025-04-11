#include "arm.hpp"

using namespace vex;

Arm::Arm(int port)
{
    motor motor_1 = motor(port);
    arm_motor = motor_group(motor_1);
    state = IDLE;
    thread(periodic);
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
    arm_motor.stop();
    state = IDLE;
}

void Arm::grab()
{
    arm_motor.spinToPosition(150.0, degrees, false);
    state = GRABBING;
}

void Arm::score()
{
    arm_motor.spinToPosition(210.0, degrees, false);
    state = SCORING;
}

void Arm::periodic()
{

    switch (state)
    {
    case IDLE:
        arm_motor.stop();
        break;
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