#include "pid_controller.h"

using namespace vex;

pid_controller::pid_controller(double kP, double kI, double kD)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    prev_error = 0;
    prev_time = 0;
    accumulated_error = 0;
}

void pid_controller::reset()
{
    prev_error = 0;
    prev_time = timer::system();
    accumulated_error = 0;
}

double pid_controller::calculate(double error)
{
    double curr_time = timer::system() / 1000;
    double dt = fmax(0.001, curr_time - prev_time);
    double d_error = (error - prev_error) / dt;
    accumulated_error += error * dt;
    prev_error = error;
    prev_time = curr_time;

    return kP * error + kI * accumulated_error + kD * d_error;
}
