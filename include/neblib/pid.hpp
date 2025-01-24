#pragma once

#include "neblib/util/optional.hpp"
#include "neblib/util/util.hpp"
#include "neblib/timer.hpp"

#include <math.h>

namespace neblib {
/**
 * @brief Struct to hold PID gains
 * 
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
struct Gains {
    double kP;
    double kI;
    double kD;

    /**
     * @brief Constructor for Gains struct
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    Gains(double kP, double kI, double kD);
};

class PID {
private:
    Gains m_gains;

    bool m_signFlipReset;
    double m_windupRange;

    double m_previousError = 0;
    double m_integral = 0;

    Optional<double> m_previousTime = Optional<double>();

public:
    /**
     * @brief Creates a PID object
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param windupRange range that the integral value will "windup"
     * @param signFlipReset if true, integral value will reset as error changes sign
     */
    PID(double kP, double kI, double kD, double windupRange, bool signFlipReset = false);

    /**
     * @brief Creates a PID object
     * 
     * @param gains Gains struct containing the gain values
     * @param windupRange range that the integral value will "windup"
     * @param signFlipReset if true, integral value will reset as error changes sign
     */
    PID(const Gains& gains, double windupRange, bool signFlipReset = false);

    /**
     * @brief Gets the gain values
     * 
     * @return Gain struct
     */
    Gains getGains();

    /**
     * @brief Sets the gain values
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    void setGains(double kP, double kI, double kD);

    /**
     * @brief Sets the gain values using a Gains struct
     * 
     * @param gains Gain struct
     */
    void setGains(const Gains& gains);

    /**
     * @brief Gets the windup range of the PID object
     * 
     * @return double
     */
    double getWindupRange();

    /**
     * @brief Sets the windup range of the PID object
     * 
     * @param windupRange range that the integral value will "windup"
     */
    void setWindupRange(double windupRange);

    /**
     * @brief Gets whether or not signFlipReset is enabled
     * 
     * @return bool
     */
    bool getSignFlipReset();

    /**
     * @brief Sets whether or not signFlipReset is enabled
     * 
     * @param signFlipReset if true, integral value will reset as error changes sign
     */
    void setSignFlipReset(bool signFlipReset);

    /**
     * @brief Resets the integral and deriative values to 0
     */
    void reset();

    /**
     * @brief Updates the output value of the PID
     * 
     * @param error the error value
     */
    double update(double error);

    /**
     * @brief Updates the output value of the PID
     * 
     * @param error the error value
     * @param minOutput the minimum expected output value
     * @param maxOutput the maximum expected outout value
     */
    double update(double error, double minOutput, double maxOutput);

    /**
     * @brief Returns true if settled, false otherwise
     */
    bool isSettled();
};

}