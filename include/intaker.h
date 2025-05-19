#pragma once
#include <vex.h>

using namespace vex;

class intaker
{
    motor_group &intake_motor;
    optical &intake_sensor;
    thread rejectThread;
    bool runIntake;

public:
    intaker(motor_group &m, optical &o);

    void periodic();
    static void _startPeriodic(void* obj);

    void intake();
    void stop();
};