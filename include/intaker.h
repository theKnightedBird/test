#pragma once
#include <vex.h>

using namespace vex;

class intaker
{
    motor_group &intake_motor;
    optical &intake_sensor;
    thread rejectThread;
    bool runIntake = false;
    bool hasRing = false;
    double numRingsInGoal = 0;

public:
    intaker(motor_group &m, optical &o);

    void periodic();
    static void _startPeriodic(void *obj);

    bool holdingRing();
    double getNumRingsInGoal();

    void resetCount();

    void intake();
    void stop();
};