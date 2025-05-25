#include "util_functions.h"

double angle_between(double target, double current)
{
    double delta = fmod((target - current + 180.0), 360.0);
    if (delta < 0)
        delta += 360.0;
    return delta - 180.0;
}
