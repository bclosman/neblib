#include "neblib/pid.hpp"

neblib::Gains::Gains(double kP, double kI, double kD) :
    kP(kP),
    kI(kI),
    kD(kD) {}

neblib::PID::PID(double kP, double kI, double kD, double windupRange, double settleTolerance, int settleTime, int iterationDelay, bool signFlipReset) :
    m_gains(Gains(kP, kI, kD)),
    m_windupRange(windupRange),
    m_settleTolerance(settleTolerance),
    m_settleTime(settleTime),
    m_iterationDelay(iterationDelay),
    m_signFlipReset(signFlipReset) {}

neblib::PID::PID(const Gains& gains, double windupRange, double settleTolerance, int settleTime, int iterationDelay, bool signFlipReset) :
    m_gains(gains),
    m_windupRange(windupRange),
    m_settleTolerance(settleTolerance),
    m_settleTime(settleTime),
    m_iterationDelay(iterationDelay),
    m_signFlipReset(signFlipReset) {}

void neblib::PID::setGains(const Gains& gains) { m_gains = gains; }

void neblib::PID::setGains(double kP, double kI, double kD) { m_gains = Gains(kP, kI, kD); }

void neblib::PID::reset()
{
    m_previousError = 0;
    m_integral = 0;
}

double neblib::PID::update(double error)
{
    const double derivative = error - m_previousError;

    m_integral += error;
    if (sign(error) != sign(m_previousError) && m_signFlipReset) m_integral = 0;
    if (abs(error) > m_windupRange) m_integral = 0;

    if (abs(error) < m_settleTolerance) m_timeSettled += m_iterationDelay;
    else m_timeSettled = 0;

    return error * m_gains.kP + m_integral * m_gains.kI + derivative * m_gains.kD;
}

double neblib::PID::update(double error, double minOutput, double maxOutput)
{
    double output = update(error);

    if (output > maxOutput) return maxOutput;
    if (output < minOutput) return minOutput;
    return output;
}

bool neblib::PID::isSettled()
{
    return (m_timeSettled >= m_settleTime);
}