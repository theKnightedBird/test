#include "intaker.h"

intaker::intaker(motor_group &m, optical &o) : intake_motor(m),
                                             intake_sensor(o),
                                             rejectThread(_startPeriodic, this)
{
}

void intaker::periodic()
{
    double hue;
    while (true)
    {
        if (runIntake)
        {
            hue = intake_sensor.hue();
            intake_motor.spin(fwd, 50, pct);
            wait(250, msec);
            // deal with jams
            if (intake_motor.velocity(pct) < 5)
            {
                intake_motor.spinFor(reverse, 100, msec);
                intake_motor.spin(forward, 50, pct);
            }

            // reject rings
            if (allianceRing == RedRing && (hue >= 340 && hue <= 359 || hue <= 20 && hue >= 0))
            {
                intake_motor.spinFor(reverse, 500, msec);
            }
            if (allianceRing == BlueRing && hue >= 200 && hue <= 230)
            {
                intake_motor.spinFor(reverse, 500, msec);
            }
        }
        else
        {
            intake_motor.stop(coast);
        }
    }
}

void intaker::_startPeriodic(void *obj)
{
    static_cast<intaker *>(obj)->periodic();
}

void intaker::intake()
{
    runIntake = true;
}
