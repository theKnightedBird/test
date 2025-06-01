#pragma once
#include "v5.h"
#include "v5_vcs.h"
#include "order.h"
#include <functional>

using namespace vex;

class subsystem
{
public:
    subsystem();

    void init();
    void periodic();
    void periodicCommand();
    static void _startPeriodic(void *obj);

    void set_order(order o);
    bool busy();

private:
    thread periodic_thread;
    thread order_thread;
    order active_order;
    bool order_active;
    mutex mtx;
};