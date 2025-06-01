#include "subsystem.h"

subsystem::subsystem() : periodic_thread(_startPeriodic, this), mtx() {}

void subsystem::periodicCommand()
{
    init();
    while (1)
    {
        mtx.lock();
        periodic();
        mtx.unlock();
        wait(20, msec);
    }
}

void _startPeriodic(void *obj)
{
    static_cast<subsystem *>(obj)->periodicCommand();
}

void subsystem::set_order(order o)
{
    order_thread.interrupt();
    std::function<void()> deal_with_prev = [&]() {};
    if (order_active)
    {
        deal_with_prev = active_order.on_interrupt;
    }
    auto order_lambda = [&]()
    {
        order_active = true;
        deal_with_prev();
        o.on_begin();
        while (!o.is_finished())
        {
            o.while_running();
            vex::wait(20, msec);
        }
        o.on_end();
        order_active = false;
    };
    // order_thread = thread(order_lambda);
}