#include "clamp.hpp"

using namespace vex;

Clamp::Clamp(triport::port port) : piston(port)
{
    state = UNCLAMP;
    thread(periodic);
}

void Clamp::clamp()
{
    state = CLAMP;
}

void Clamp::unclamp()
{
    state = UNCLAMP;
}

void Clamp::periodic()
{

    switch (state)
    {
    case CLAMP:
        piston.set(true);
        break;
    case UNCLAMP:
        piston.set(false);
        break;
    default:
        break;
    }
}
