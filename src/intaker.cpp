#include "intaker.h"

intaker::intaker(motor_group &m, optical &o) : intake_motor(m),
                                               intake_sensor(o),
                                               periodicThread(_startPeriodic, this)
{
}

void intaker::periodic()
{
    double hue;
    while (true)
    {
        hue = intake_sensor.hue();

        // // ring owning logic
        // if (allianceRing == RedRing && hue >= 200 && hue <= 230)
        //     hasRing = true;
        // else if (allianceRing == BlueRing && ((hue >= 340 && hue <= 359) || (hue <= 20 && hue >= 0)))
        //     hasRing = true;
        // else
        // {
        //     if (hasRing == true)
        //         numRingsInGoal++;
        //     hasRing = false;
        // }

        // logic for intaking
        if (runIntake)
        {
            intake_motor.spin(fwd, 100, pct);
            wait(250, msec);
            // // deal with jams
            // if (intake_motor.velocity(pct) < 5)
            // {
            //     intake_motor.spinFor(reverse, 100, msec);
            //     intake_motor.spin(forward, 50, pct);
            // }

            // // reject rings
            // if (allianceRing == RedRing && ((hue >= 340 && hue <= 359) || (hue <= 20 && hue >= 0)))
            // {
            //     intake_motor.spinFor(reverse, 500, msec);
            // }
            // if (allianceRing == BlueRing && hue >= 200 && hue <= 230)
            // {
            //     intake_motor.spinFor(reverse, 500, msec);
            // }
        }
        else
        {
            intake_motor.stop(brake);
        }
        wait(20, msec);
    }
}

void intaker::_startPeriodic(void *obj)
{
    static_cast<intaker *>(obj)->periodic();
}

bool intaker::holdingRing()
{
    return hasRing;
}

double intaker::getNumRingsInGoal()
{
    return numRingsInGoal;
}

void intaker::resetCount()
{
    numRingsInGoal = 0;
}

void intaker::intake()
{
    runIntake = true;
}

void intaker::stop()
{
    runIntake = false;
}
