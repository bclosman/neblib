#include "neblib/pid.hpp"

neblib::PID::Gains::Gains(const double kP, const double kI, const double kD, const bool resetIntegralOnPass, const double windupRange) :
    kP(kP),
    kI(kI),
    kD(kD),
    resetIntegralOnPass(resetIntegralOnPass),
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
        if (std::abs(error) <= tolerance) SettleTime.timeSettled += timeStep;
        else SettleTime.timeSettled = 0;
        return SettleTime.timeSettled >= SettleTime.settleTime;
    } else {
        return (std::abs(error) <= tolerance) && (std::abs(derivative) <= Derivative.tolerance);
    }
}

neblib::PID::PID(const neblib::PID::Gains& gains, const neblib::PID::ExitConditions& exitConditions) :
    gains(gains),
    exitConditions(exitConditions) {}

neblib::PID::PID(neblib::PID::Gains&& gains, neblib::PID::ExitConditions&& exitConditions) :
    gains(gains),
    exitConditions(exitConditions) {}

double neblib::PID::getOutput(const double error, const double maxOutput, const double minOutput) {
    if (std::abs(error) <= gains.windupRange) integral += error;
    else integral = 0;
    if (gains.resetIntegralOnPass && sign(error) != sign(previousError)) integral = 0;

    double derivative = error - previousError;

    settled = exitConditions.isSettled(error, derivative);

    double output = gains.kP * error + gains.kI * integral + gains.kD * derivative;

    if (output > maxOutput) return maxOutput;
    else if (output < minOutput) return minOutput;
    else return output;
}

double neblib::PID::getOutput(const double error) {
    return getOutput(error, INFINITY, -INFINITY);
}

bool neblib::PID::isSettled() { return settled; }