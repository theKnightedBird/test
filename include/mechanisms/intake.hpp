#pragma once
#include <vex.h>

enum intake_state {
    INTAKE,
    OUTTAKE,
    HOLD
};

class Intake {
    public:
        Intake(int intake, int conveyor);
        void intake();
        void outtake();
        void hold();
        void periodic();
        intake_state getState();
    private:
        vex::motor_group intake_motor;
        intake_state state;
};