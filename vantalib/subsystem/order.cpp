#include "order.h"

order::order(
    function<void()> on_begin,
    function<void()> while_running,
    function<bool()> is_finished,
    function<void()> on_end,
    function<void()> on_interrupt) : on_begin(on_begin),
                                     while_running(while_running),
                                     is_finished(is_finished),
                                     on_end(on_end),
                                     on_interrupt(on_interrupt) {}