#include "intake.hpp"

Intake::Intake(int intake, int conveyor) {
    intake_motor(vex::motor(intake), vex::motor(conveyor));
    state = HOLD;
}

void Intake::intake() {
    state = INTAKE;
}

void Intake::outtake() {
    state = OUTTAKE;
}

void Intake::hold() {
    state = HOLD;
}

void Intake::periodic() {
    switch (state) {
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