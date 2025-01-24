#pragma once

#include <chrono>

namespace neblib {

enum class TimeUnits {
    millisecond = 0,
    ms = 0,
    second = 1,
    s = 1
};

class Timer {
    /**
     * @brief Class used to access the VEX Brain's timer without needing a vex::brain object.
     * More or less a simpler way to call std::chrono
     */
public:
    /**
     * @brief Gets the time in a specified unit
     * 
     * @param unit the desired time unit
     * 
     * @return double
     */
    static double getTime(TimeUnits unit = TimeUnits::ms);

    /**
     * @brief Converts between two time units
     * 
     * @param value the original time value
     * @param originalUnit the unit of the original time value
     * @param newUnit the desired unit
     * 
     * @return double
     */
    static double convert(double value, TimeUnits originalUnit, TimeUnits newUnit);
};
}