#pragma once

#include "neblib/util/util.hpp"

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

    double m_windupRange;
    double m_settleTolerance;
    int m_settleTime;
    int m_iterationDelay;
    bool m_signFlipReset;

    double m_previousError = 0;
    double m_integral = 0;
    int m_timeSettled = 0;
    
public:
    /**
     * @brief Creates a PID object
     * 
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param windupRange range that the integral value will "windup"
     * @param settleTolerance range where the PID will be considered "settled"
     * @param settleTime the length of time (in milliseconds) that the error needs to be within the settle tolerance
     * @param iterationDelay the length of time (in milliseconds) that your loop's delay is
     * @param signFlipReset if true, integral value will reset as error changes sign
     */
    PID(double kP, double kI, double kD, double windupRange, double settleTolerance, int settleTime, int iterationDelay, bool signFlipReset = false);

    /**
     * @brief Creates a PID object
     * 
     * @param gains Gains struct containing the gain values
     * @param windupRange range that the integral value will "windup"
     * @param settleTolerance range where the PID will be considered "settled"
     * @param settleTime the length of time (in milliseconds) that the error needs to be within the settle tolerance
     * @param iterationDelay the length of time (in milliseconds) that your loop's delay is
     * @param signFlipReset if true, integral value will reset as error changes sign
     */
    PID(const Gains& gains, double windupRange, double settleTolerance, int settleTime, int iterationDelay, bool signFlipReset = false);

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