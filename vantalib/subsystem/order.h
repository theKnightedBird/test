#pragma once
#include <functional>

using namespace std;

class order
{
public:
    order();
    order(
        function<void()> on_begin,
        function<void()> while_running,
        function<bool()> is_finished,
        function<void()> on_end,
        function<void()> on_interrupt);
    function<void()> on_begin;
    function<void()> while_running;
    function<bool()> is_finished;
    function<void()> on_end;
    function<void()> on_interrupt;
};