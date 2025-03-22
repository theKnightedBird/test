#pragma once
#include <vex.h>

enum clamp_state
{
    CLAMP,
    UNCLAMP
};

class Clamp
{
public:
    Clamp(triport::port port);
    void clamp();
    void unclamp();
    void periodic();
    clamp_state getState();

private:
    vex::digital_out piston;
    clamp_state state;
};