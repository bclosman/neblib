#include "neblib/pid.hpp"

neblib::PID::Gains::Gains(const double kP, const double kI, const double kD, const double windupRange) :
    kP(kP),
    kI(kI),
    kD(kD),
    windupRange(windupRange) {}

neblib::PID::ExitConditions::ExitConditions(const neblib::PID::ExitConditions::Type type, const double tolerance, const int timeStep, const double requirement) :
    type(type),
    tolerance(tolerance),
    timeStep(timeStep) {
        if (type == SETTLE_TIME) SettleTime.settleTime = int(requirement);
        else Derivative.tolerance = requirement;
    }

bool neblib::PID::ExitConditions::isSettled(const double error, const double derivative) {
    if (type == SETTLE_TIME) {
        if (abs(error) <= tolerance) SettleTime.timeSettled += timeStep;
        return SettleTime.timeSettled >= SettleTime.settleTime;
    } else {
        return (abs(error) <= tolerance) && (abs(derivative) <= Derivative.tolerance);
    }
}