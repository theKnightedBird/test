#pragma once
#include <math.h>

class vec2
{
public:
    double x;
    double y;
    vec2();
    vec2(double x, double y);
    double mag();
    vec2 operator+(vec2 v);
    vec2 operator-(vec2 v);
    vec2 operator*(double s);
};

double dot(vec2 v1, vec2 v2);

double distance_between(vec2 v1, vec2 v2);

double distance_from_line_segment(vec2 start, vec2 end, vec2 point);
double angle_between(double target, double current);
