#pragma once
#include "vex.h"

using namespace vex;

class pid_controller
{
    double kP;
    double kI;
    double kD;
    double prev_error;
    double prev_time;
    double accumulated_error;

public:
    pid_controller(double kP, double kI, double kD);
    void reset();
    bool inTolerance();
    double calculate(double error);
};