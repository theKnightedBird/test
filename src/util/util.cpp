#include "util.h"

vec2::vec2(double x, double y)
{
    this->x = x;
    this->y = y;
}
double vec2::mag()
{
    return sqrt(x * x + y * y);
}

vec2 vec2::operator+(vec2 v)
{
    return vec2(x + v.x, y + v.y);
}

vec2 vec2::operator-(vec2 v)
{
    return vec2(x - v.x, y - v.y);
}

vec2 vec2::operator*(double s)
{
    return vec2(s * x, s * y);
}

double dot(vec2 v1, vec2 v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

double distance_between(vec2 v1, vec2 v2)
{
    return sqrt(pow((v1.x - v2.x), 2) + pow((v1.y - v2.y), 2));
}

double distance_from_line_segment(vec2 start, vec2 end, vec2 point)
{
    if (distance_between(start, end) == 0)
        return distance_between(start, point);

    vec2 line = end - start;
    vec2 point_trans = point - start;
    vec2 projection = line * (dot(point_trans, line) / pow(line.mag(), 2));
    if (dot(point_trans, line) / pow(line.mag(), 2) <= 0) // extends past start
    {
        return distance_between(start, point);
    } else if (dot(point_trans, line) / pow(line.mag(), 2) >= 1) // extends past end
    {
        return distance_between(end, point);
    } else {
        return (point_trans - projection).mag();
    }
}

double angle_between(double target, double current)
{
    double delta = fmod((target - current + 180.0), 360.0);
    if (delta < 0)
        delta += 360.0;
    return delta - 180.0;
}
