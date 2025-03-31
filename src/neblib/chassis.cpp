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
    while (!drivePID.isSettled() && time < timeout) {
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

void neblib::Chassis::setHeading(double heading) { IMU->setHeading(heading, vex::rotationUnits::deg); }

int neblib::Chassis::driveFor(double distance, int timeout, double heading, double minOutput, double maxOutput) {
    return driveFor(distance, timeout, heading, minOutput, maxOutput, drivePIDConstants.gains, drivePIDConstants.headingGains, drivePIDConstants.exitConditions);
}

int neblib::Chassis::driveFor(double distance, int timeout, double minOutput, double maxOutput) {
    return driveFor(distance, timeout, IMU->heading(vex::rotationUnits::deg), minOutput, maxOutput, drivePIDConstants.gains, drivePIDConstants.headingGains, drivePIDConstants.exitConditions);
}

int neblib::Chassis::driveFor(double distance, int timeout, double heading) {
    return driveFor(distance, timeout, heading, -12, 12, drivePIDConstants.gains, drivePIDConstants.headingGains, drivePIDConstants.exitConditions);
}

int neblib::Chassis::driveFor(double distance, int timeout) {
    return driveFor(distance, timeout, IMU->heading(vex::rotationUnits::deg), -12, 12, drivePIDConstants.gains, drivePIDConstants.headingGains, drivePIDConstants.exitConditions);
}

int neblib::Chassis::driveFor(double distance) {
    return driveFor(distance, INFINITY, IMU->heading(vex::rotationUnits::deg), -12, 12, drivePIDConstants.gains, drivePIDConstants.headingGains, drivePIDConstants.exitConditions);
}

int neblib::Chassis::turnFor(double degrees, int timeout, double minOutput, double maxOutput, neblib::PID::Gains gains, neblib::PID::ExitConditions exitConditions) {
    neblib::PID turnPID = neblib::PID(gains, exitConditions);

    double startingPosition = IMU->rotation(vex::rotationUnits::deg);
    int time = 0;
    while(!turnPID.isSettled() && time < timeout) {
        double currentPosition = IMU->rotation(vex::rotationUnits::deg) - startingPosition;
        double output = turnPID.getOutput(degrees - currentPosition, maxOutput, minOutput);

        leftMotorGroup->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        rightMotorGroup->spin(vex::directionType::rev, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 10;
    }

    return time;
}

int neblib::Chassis::turnFor(double degrees, int timeout, double minOuput, double maxOutput) {
    return turnFor(degrees, timeout, minOuput, maxOutput, turnPIDConstants.gains, turnPIDConstants.exitConditions);
}

int neblib::Chassis::turnFor(double degrees, int timeout) {
    return turnFor(degrees, timeout, -12, 12, turnPIDConstants.gains, turnPIDConstants.exitConditions);
}

int neblib::Chassis::turnFor(double degrees) {
    return turnFor(degrees, INFINITY, -12, 12, turnPIDConstants.gains, turnPIDConstants.exitConditions);
}

int neblib::Chassis::turnTo(double heading, int timeout, double minOutput, double maxOutput, neblib::PID::Gains gains, neblib::PID::ExitConditions exitConditions) {
    return turnFor(neblib::restrain(heading - IMU->heading(vex::rotationUnits::deg), -180, 180), timeout, minOutput, maxOutput, gains, exitConditions);
}

int neblib::Chassis::turnTo(double heading, int timeout, double minOutput, double maxOutput) {
    return turnFor(neblib::restrain(heading - IMU->heading(vex::rotationUnits::deg), -180, 180), timeout, minOutput, maxOutput, turnPIDConstants.gains, turnPIDConstants.exitConditions);
}

int neblib::Chassis::turnTo(double heading, int timeout) { 
    return turnFor(neblib::restrain(heading - IMU->heading(vex::rotationUnits::deg), -180, 180), timeout, -12, 12, turnPIDConstants.gains, turnPIDConstants.exitConditions);
}

int neblib::Chassis::turnTo(double heading) {
    return turnFor(neblib::restrain(heading - IMU->heading(vex::rotationUnits::deg), -180, 180), INFINITY, -12, 12, turnPIDConstants.gains, turnPIDConstants.exitConditions);
}

int neblib::Chassis::swingFor(vex::turnType direction, double degrees, int timeout, double minOutput, double maxOutput, neblib::PID::Gains gains, neblib::PID::ExitConditions exitConditions) {
    PID swingPID = PID(gains, exitConditions);

    int multiplier = (direction == vex::turnType::right) ? 1 : -1;
    double startingPosition = IMU->rotation(vex::rotationUnits::deg);
    double target = startingPosition + (multiplier * degrees);

    int time = 0;
    while (!swingPID.isSettled()) {
        double currentPositon = IMU->rotation(vex::rotationUnits::deg) - startingPosition;
        double output = swingPID.getOutput(target - currentPositon, maxOutput, minOutput);

        if (direction == vex::turnType::right) {
            leftMotorGroup->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
            rightMotorGroup->stop(vex::brakeType::hold);
        } else {
            leftMotorGroup->stop(vex::brakeType::hold);
            rightMotorGroup->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        }

        vex::task::sleep(10);
        time += 10;
    }

    return time;
}

int neblib::Chassis::swingFor(vex::turnType direction, double degrees, int timeout, double minOuput, double maxOutput) {
    return swingFor(direction, degrees, timeout, minOuput, maxOutput, swingPIDConstants.gains, swingPIDConstants.exitConditions);
}

int neblib::Chassis::swingFor(vex::turnType direction, double degrees, int timeout) {
    return swingFor(direction, degrees, timeout, -12, 12, swingPIDConstants.gains, swingPIDConstants.exitConditions);
}

int neblib::Chassis::swingFor(vex::turnType direction, double degrees) {
    return swingFor(direction, degrees, INFINITY, -12, 12, swingPIDConstants.gains, swingPIDConstants.exitConditions);
}