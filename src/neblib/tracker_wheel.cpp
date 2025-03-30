#include "neblib/tracker_wheel.hpp"

neblib::TrackerWheel::TrackerWheel(vex::rotation& rotation, const double wheelDiameter, const double ratio) :
    rotation(&rotation),
    motor(nullptr),
    motorGroup(nullptr),
    encoder(nullptr),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

neblib::TrackerWheel::TrackerWheel(vex::motor& motor, const double wheelDiameter, const double ratio) :
    rotation(nullptr),
    motor(&motor),
    motorGroup(nullptr),
    encoder(nullptr),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

neblib::TrackerWheel::TrackerWheel(vex::motor_group& motorGroup, const double wheelDiameter, const double ratio) :
    rotation(nullptr),
    motor(nullptr),
    motorGroup(&motorGroup),
    encoder(nullptr),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

neblib::TrackerWheel::TrackerWheel(vex::encoder& encoder, const double wheelDiameter, const double ratio) :
    rotation(nullptr),
    motor(nullptr),
    motorGroup(nullptr),
    encoder(&encoder),
    distancePerDegree((ratio * (3.1415926535 * wheelDiameter)) / 360) {}

double neblib::TrackerWheel::getDistance() {
    double position = 0;
    if (rotation) position = rotation->position(vex::rotationUnits::deg);
    else if (motor) position = motor->position(vex::rotationUnits::deg);
    else if (motorGroup) position = motorGroup->position(vex::rotationUnits::deg);
    else if (encoder) position = encoder->position(vex::rotationUnits::deg);
    return position * distancePerDegree;
}