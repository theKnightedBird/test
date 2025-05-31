#include "util.h"

double distance_from_line_segment(vec start, vec end, vec point)
{
    if (vec::dist_between(start, end) < 1e-8)
        return vec::dist_between(start, point);

    vec line = end - start;
    vec point_trans = point - start;
    if (point_trans * line / line.mag() <= 0) // extends past start
    {
        return vec::dist_between(start, point);
    }
    else if (point_trans * line / line.mag() >= 1) // extends past end
    {
        return vec::dist_between(end, point);
    }
    else
    {
        return (point_trans - vec::proj(point_trans, line)).mag();
    }
}

double angle_between(double target, double current)
{
    double delta = fmod((target - current + 180.0), 360.0);
    if (delta < 0)
        delta += 360.0;
    return delta - 180.0;
}
