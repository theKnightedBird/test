#include "intake.hpp"

using namespace vex;

Intake::Intake(int intake, int conveyor)
{
    motor motor_1 = motor(intake);
    motor motor_2 = motor(conveyor);
    intake_motor = motor_group(motor_1, motor_2);
    state = HOLD;
}

void Intake::intake()
{
    intake_motor.spin(vex::fwd);
    state = INTAKE;
}

void Intake::outtake()
{
    intake_motor.spin(vex::reverse);
    state = OUTTAKE;
}

void Intake::hold()
{
    intake_motor.stop();
    state = HOLD;
}

void Intake::periodic()
{

    switch (state)
    {
    case INTAKE:
        intake_motor.spin(vex::fwd);
        break;
    case OUTTAKE:
        intake_motor.spin(vex::reverse);
        break;
    case HOLD:
        intake_motor.stop();
        break;
    default:
        intake_motor.stop();
        break;
    }
}