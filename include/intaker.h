#pragma once
#include <vex.h>

using namespace vex;

enum intake_state
{
    RUN,
    STOP,
    REVERSE
};

class intaker
{
    motor_group &intake_motor;
    optical &intake_sensor;
    thread periodicThread;
    intake_state runIntake = STOP;
    bool hasRing = false;
    double numRingsInGoal = 0;
    int hook = 980;
    int sensor_dist;
    int eject_dist;
    int nextPos;

public:
    intaker(motor_group &m, optical &o, int h, int sensor_dist, int eject_dist);

    void periodic();
    static void _startPeriodic(void *obj);

    bool holdingRing();
    double getNumRingsInGoal();

    void resetCount();

    void intake();
    void rejectRing();
    void stop();
    void reverse();
};