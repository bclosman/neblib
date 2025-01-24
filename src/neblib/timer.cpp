#include "neblib/timer.hpp"

double neblib::Timer::getTime(TimeUnits unit)
{
    double time = (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    if (static_cast<int>(unit) == 1) time *= 0.001;

    return time;
}

double neblib::Timer::convert(double value, TimeUnits originalUnit, TimeUnits newUnit)
{   
    double multiplier = 1;
    if (static_cast<int>(originalUnit) > static_cast<int>(newUnit))
    {
        for (int i = static_cast<int>(originalUnit); i > static_cast<int>(newUnit); i--) multiplier *= 1000;
    }
    else if (static_cast<int>(originalUnit) < static_cast<int>(newUnit))
    {
        for (int i = static_cast<int>(originalUnit); i < static_cast<int>(newUnit); i++) multiplier *= 0.001;
    }

    return multiplier * value;
}
