#pragma once

#include "vex.h"

namespace neblib {

class TrackerWheel {
private:
    const vex::rotation* rotation;
    const vex::motor* motor;
    const vex::motor_group* motorGroup;
    const vex::encoder* encoder;
    
    double distancePerDegree;

public:
    /// @brief Creates a TrackerWheel object using a VEX V5 Rotation sensor
    /// @param rotation a VEX V5 Rotation sensor
    /// @param wheelDiameter diameter of the wheel
    /// @param ratio ratio, generally sensor gear teeth / wheel sensor teeth
    TrackerWheel(const vex::rotation& rotation, const double wheelDiameter, const double ratio);

    /// @brief Creates a TrackerWheel object using a VEX V5 Motor
    /// @param motor a VEX V5 Motor
    /// @param wheelDiameter diameter of the wheel
    /// @param ratio ratio, generally sensor gear teeth / wheel sensor teeth
    TrackerWheel(const vex::motor& motor, const double wheelDiameter, const double ratio);

    /// @brief Creates a TrackerWheel object using a VEX V5 Motor group
    /// @param motorGroup a VEX V5 Motor group
    /// @param wheelDiameter diameter of the wheel
    /// @param ratio ratio, generally sensor gear teeth / wheel sensor teeth
    TrackerWheel(const vex::motor_group& motorGroup, const double wheelDiameter, const double ratio);

    /// @brief Creates a TrackerWheel object using a VEX Legacy OS Encoder
    /// @param encoder a VEX Legacy OS Encoder
    /// @param wheelDiameter diameter of the wheel
    /// @param ratio ratio, generally sensor gear teeth / wheel sensor teeth
    TrackerWheel(const vex::encoder& encoder, const double wheelDiameter, const double ratio);


    /// @brief Gets the distance of the tracker wheel
    double getDistance();
};

};