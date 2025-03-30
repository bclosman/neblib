#include "neblib/tracker_wheel.hpp"

neblib::TrackerWheel::TrackerWheel(const vex::rotation& rotation, const double wheelDiameter, const double ratio) :
    rotation(&rotation),
    motor(nullptr),
    motorGroup(nullptr),
    encoder(nullptr),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

neblib::TrackerWheel::TrackerWheel(const vex::motor& motor, const double wheelDiameter, const double ratio) :
    rotation(nullptr),
    motor(&motor),
    motorGroup(nullptr),
    encoder(nullptr),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

neblib::TrackerWheel::TrackerWheel(const vex::motor_group& motorGroup, const double wheelDiameter, const double ratio) :
    rotation(nullptr),
    motor(nullptr),
    motorGroup(&motorGroup),
    encoder(nullptr),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

neblib::TrackerWheel::TrackerWheel(const vex::encoder& encoder, const double wheelDiameter, const double ratio) :
    rotation(nullptr),
    motor(nullptr),
    motorGroup(nullptr),
    encoder(&encoder),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}