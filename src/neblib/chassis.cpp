#include "neblib/chassis.hpp"

neblib::Chassis::Chassis(vex::motor_group& leftMotorGroup, vex::motor_group& rightMotorGroup, neblib::TrackerWheel& trackerWheel, vex::inertial& IMU, double trackWidth) :
    leftMotorGroup(&leftMotorGroup),
    rightMotorGroup(&rightMotorGroup),
    trackerWheel(trackerWheel),
    IMU(&IMU),
    trackWidth(trackWidth) {}

void neblib::Chassis::setDriveConstants(neblib::PID::Gains&& driveGains, neblib::PID::Gains&& turnGains, neblib::PID::ExitConditions&& exitConditions) {
    drivePIDConstants.gains = driveGains;
    drivePIDConstants.headingGains = turnGains;
    drivePIDConstants.exitConditions = exitConditions;
}

void neblib::Chassis::setTurnConstants(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions) {
    turnPIDConstants.gains = gains;
    turnPIDConstants.exitConditions = exitConditions;
}

void neblib::Chassis::setSwingConstants(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions) {
    swingPIDConstants.gains = gains;
    swingPIDConstants.exitConditions = exitConditions;
}

void neblib::Chassis::setArcConstants(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions) {
    arcPIDConstants.gains = gains;
    arcPIDConstants.exitConditions = exitConditions;
}

neblib::PID::Gains neblib::Chassis::getDriveGains() { return drivePIDConstants.gains; }

neblib::PID::Gains neblib::Chassis::getDriveHeadingGains() { return drivePIDConstants.headingGains; }

neblib::PID::Gains neblib::Chassis::getTurnGains() { return turnPIDConstants.gains; }

neblib::PID::Gains neblib::Chassis::getSwingGains() { return swingPIDConstants.gains; }

neblib::PID::Gains neblib::Chassis::getArcGains() { return arcPIDConstants.gains; }

neblib::PID::ExitConditions neblib::Chassis::getDriveExitConditions() { return drivePIDConstants.exitConditions; }

neblib::PID::ExitConditions neblib::Chassis::getTurnExitConditions() { return turnPIDConstants.exitConditions; }

neblib::PID::ExitConditions neblib::Chassis::getSwingExitConditions() { return swingPIDConstants.exitConditions; }

neblib::PID::ExitConditions neblib::Chassis::getArcExitConditions() { return arcPIDConstants.exitConditions; }

int neblib::Chassis::driveFor(double distance, int timeout, double heading, double minOutput, double maxOutput, neblib::PID::Gains driveGains, neblib::PID::Gains turnGains, neblib::PID::ExitConditions exitConditions) {
    neblib::PID drivePID = neblib::PID(driveGains, exitConditions);
    neblib::PID turnPID = neblib::PID(turnGains, exitConditions);

    double startingPositon = trackerWheel.getDistance();
    int time = 0;
    while (!drivePID.isSettled()) {
        double currentPosition = trackerWheel.getDistance() - startingPositon;

        double driveOutput = drivePID.getOutput(distance - currentPosition, maxOutput, minOutput);
        double turnOutput = turnPID.getOutput(restrain(heading - IMU->heading(vex::rotationUnits::deg), -180, 180), maxOutput, minOutput);

        leftMotorGroup->spin(vex::directionType::fwd, clamp(driveOutput + turnOutput, -12, 12), vex::voltageUnits::volt);
        rightMotorGroup->spin(vex::directionType::fwd, clamp(driveOutput - turnOutput, -12, 12), vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 10;
    }
    return time;
}