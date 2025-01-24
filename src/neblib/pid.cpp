#include "neblib/pid.hpp"

neblib::Gains::Gains(double kP, double kI, double kD) :
    kP(kP),
    kI(kI),
    kD(kD) {}

neblib::PID::PID(double kP, double kI, double kD, double windupRange, bool signFlipReset) :
    m_gains(Gains(kP, kI, kD)),
    m_signFlipReset(signFlipReset),
    m_windupRange(windupRange) {}

neblib::PID::PID(const Gains &gains, double windupRange, bool signFlipReset) :
    m_gains(gains),
    m_signFlipReset(signFlipReset),
    m_windupRange(windupRange) {}

neblib::Gains neblib::PID::getGains()
{
    return m_gains;
}

void neblib::PID::setGains(double kP, double kI, double kD)
{
    m_gains = Gains(kP, kI, kD);
}

void neblib::PID::setGains(const Gains &gains)
{
    m_gains = gains;
}

double neblib::PID::getWindupRange()
{
    return m_windupRange;
}

void neblib::PID::setWindupRange(double windupRange)
{
    m_windupRange = windupRange;
}

bool neblib::PID::getSignFlipReset()
{
    return m_signFlipReset;
}

void neblib::PID::setSignFlipReset(bool signFlipReset)
{
    m_signFlipReset = signFlipReset;
}

void neblib::PID::reset()
{
    m_previousError = 0;
    m_integral = 0;
}

double neblib::PID::update(double error)
{
    const double now = Timer::getTime(TimeUnits::second);
    const double deltaTime = (!m_previousTime.hasValue()) ? 0.0 : now - m_previousTime.value();
    m_previousTime.setValue(now);

    const double derivative = (deltaTime != 0.0) ? (error - m_previousError) / deltaTime : 0;

    m_integral += error * deltaTime;
    if (sign(error) != sign(m_previousError) && m_signFlipReset) m_integral = 0;
    if (fabs(error) > m_windupRange) m_integral = 0;

    return error * m_gains.kP + m_integral * m_gains.kI + derivative * m_gains.kD;
}