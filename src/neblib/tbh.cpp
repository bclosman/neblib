#include "neblib/tbh.hpp"

neblib::TBH::TBH(double gain) : m_gain(gain) {}

void neblib::TBH::setGain(double gain)
{
    m_gain = gain;
}

double neblib::TBH::getGain()
{
    return m_gain;
}

/**
 * @todo test and figure this out
 * 
 * Current Attempt: 1
 * Attempt 1 started: 01/24/2025, Bryce Closman
 */
double neblib::TBH::update(double error)
{
    const double now = Timer::getTime(TimeUnits::second);
    const double deltaTime = (!m_previousTime.hasValue()) ? 0 : now - m_previousTime.value();
    m_previousTime.setValue(now);

    m_buildup += error * deltaTime;
    if (sign(error) != sign(m_previousError))
    {
        m_buildup = (m_buildup + m_buildupResetValue) / 2;
        m_buildupResetValue = m_buildup;
    }

    return m_buildup;
}

void neblib::TBH::reset()
{
    m_buildup = 0;
    m_buildupResetValue = 0;
    m_previousError = 0;
}

void neblib::TBH::setResetValue(double resetValue)
{
    m_buildupResetValue = resetValue;
}

double neblib::TBH::getResetValue()
{
    return m_buildupResetValue;
}
