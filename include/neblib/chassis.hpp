#pragma once

#include "neblib/pid.hpp"
#include "neblib/tracker_wheel.hpp"
#include "vex.h"

namespace neblib {

class Chassis {
private:
    vex::motor_group* leftMotorGroup;
    vex::motor_group* rightMotorGroup;

    neblib::TrackerWheel trackerWheel;
    vex::inertial* IMU;

    double trackWidth;

    struct {
       neblib::PID::Gains gains = neblib::PID::Gains(0, 0, 0, true, 0);
       neblib::PID::Gains headingGains = neblib::PID::Gains(0, 0, 0, true, 0);
       neblib::PID::ExitConditions exitConditions = neblib::PID::ExitConditions(neblib::PID::ExitConditions::Type::SETTLE_TIME, 0, 0, 0); 
    } drivePIDConstants;

    struct {
        neblib::PID::Gains gains = neblib::PID::Gains(0, 0, 0, true, 0);
        neblib::PID::ExitConditions exitConditions = neblib::PID::ExitConditions(neblib::PID::ExitConditions::Type::SETTLE_TIME, 0, 0, 0); 
    } turnPIDConstants;

    struct {
        neblib::PID::Gains gains = neblib::PID::Gains(0, 0, 0, true, 0);
        neblib::PID::ExitConditions exitConditions = neblib::PID::ExitConditions(neblib::PID::ExitConditions::Type::SETTLE_TIME, 0, 0, 0); 
    } swingPIDConstants;

    struct {
        neblib::PID::Gains gains = neblib::PID::Gains(0, 0, 0, true, 0);
        neblib::PID::ExitConditions exitConditions = neblib::PID::ExitConditions(neblib::PID::ExitConditions::Type::SETTLE_TIME, 0, 0, 0); 
    } arcPIDConstants;

public:
    Chassis(vex::motor_group& leftMotorGroup, vex::motor_group& rightMotorGroup, neblib::TrackerWheel& TrackerWheel, vex::inertial& IMU, double trackWidth);

    void setDriveConstants(neblib::PID::Gains&& driveGains, neblib::PID::Gains&& turnGains, neblib::PID::ExitConditions&& exitConditions);
    void setTurnConstants(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions);
    void setSwingConstants(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions);
    void setArcConstants(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions);

    neblib::PID::Gains getDriveGains();
    neblib::PID::Gains getDriveHeadingGains();
    neblib::PID::Gains getTurnGains();
    neblib::PID::Gains getSwingGains();
    neblib::PID::Gains getArcGains();

    neblib::PID::ExitConditions getDriveExitConditions();
    neblib::PID::ExitConditions getTurnExitConditions();
    neblib::PID::ExitConditions getSwingExitConditions();
    neblib::PID::ExitConditions getArcExitConditions();

    int driveFor(double distance, int timeout, double heading, double minOutput, double maxOutput, neblib::PID::Gains driveGains, neblib::PID::Gains turnGains, neblib::PID::ExitConditions exitConditions);
};

}