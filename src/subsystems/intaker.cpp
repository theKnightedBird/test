#include "intaker.h"

intaker::intaker(motor_group &m, optical &o, int h, int sensor_d, int eject_d) : intake_motor(m),
                                                                                 intake_sensor(o),
                                                                                 periodicThread(_startPeriodic, this),
                                                                                 runIntake(false),
                                                                                 hook(h),
                                                                                 sensor_dist(sensor_d),
                                                                                 eject_dist(eject_d)
{
    intake_sensor.setLightPower(100, percent);
    intake_sensor.setLight(ledState::on);
#if !defined(MANAGER_ROBOT)
    intake_motor.setPosition(-360, degrees);
#endif
}

void intaker::rejectRing()
{
#if defined(MANAGER_ROBOT)
    double pos = intake_motor.position(degrees);
    nextPos = ((int)((pos + sensor_dist / hook) * hook + eject_dist));
    while (pos < nextPos)
    {
        pos = intake_motor.position(degrees);
        wait(10, msec);
    }
    intake_motor.spin(reverse);
    wait(100, msec);
    intake_motor.spin(forward);
#else
    double pos = intake_motor.position(degrees);
    double nextPos = ((int)(pos / hook) + 1) * hook;

    while (pos < nextPos)
    {
        pos = intake_motor.position(degrees);
        wait(5, msec);
    }
    intake_motor.stop();
    wait(100, msec);
    intake_motor.spin(forward);
#endif
}

void intaker::periodic()
{
    double hue;
    while (true)
    {
        hue = intake_sensor.hue();
        if (runIntake)
        {
            intake_motor.spin(fwd, 90, pct);
            // // deal with jams
            // if (intake_motor.velocity(pct) < 5)
            // {
            //     intake_motor.spinFor(reverse, 100, msec);
            //     intake_motor.spin(forward, 50, pct);
            // }

            // reject rings
            if (allianceRing == BlueRing && ((hue >= 340 && hue <= 359) || (hue <= 20 && hue >= 0)))
            {
                rejectRing();
            }
            if (allianceRing == RedRing && hue >= 190 && hue <= 230)
            {
                rejectRing();
            }
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
