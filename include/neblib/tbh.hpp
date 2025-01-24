#pragma once

#include "util/util.hpp"
#include "util/optional.hpp"
#include "timer.hpp"

namespace neblib {

class TBH {
    /**
     * @brief Modified take back half controller
     */
public:
    /**
     * @brief Creates a TBH object
     * 
     * @param gain the gain value
     */
    TBH(double gain);

    /**
     * @brief Sets the gain value
     * 
     * @param gain the gain value
     */
    void setGain(double gain);

    /**
     * @brief Gets the gain value
     */
    double getGain();

    /**
     * @brief Updates the TBH object and returns an output
     * 
     * @param error the current error
     */
    double update(double error);

    /**
     * @brief Resets values to zero
     */
    void reset();

    /**
     * @brief Sets the buildup reset value
     * 
     * @param resetValue the new reset value
     */
    void setResetValue(double resetValue);

    /**
     * @brief Gets the current buildup reset value
     */
    double getResetValue();

private:
    double m_gain;

    double m_buildup = 0;
    double m_buildupResetValue = 0;
    double m_previousError = 0;
    Optional<double> m_previousTime = Optional<double>();
};

}