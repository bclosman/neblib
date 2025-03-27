#pragma once

#include "neblib/util/util.hpp"

#include <math.h>

namespace neblib {

/// @brief PID Controller
class PID {
public:

    /// @brief Gains (P, I, D) for PID object
    struct Gains {
        double kP;
        double kI;
        double kD;
        double windupRange;


        /// @brief Constructor for Gains struct
        /// @param kP proportional constant
        /// @param kI integral constant
        /// @param kD derivative constant 
        /// @param windupRange range where integration takes place
        Gains(const double kP, const double kI, const double kD, const double windupRange = INFINITY);
    };

    /// @brief Exit conditions for PID object
    struct ExitConditions {
        /// @brief Type of exit condition used
        enum Type {
            SETTLE_TIME,
            DERIVATIVE
        };

        Type type;
        double tolerance;
        int timeStep;

        /// @brief Struct for the settle time method
        struct {
            int settleTime;
            int timeSettled = 0;
        } SettleTime;

        /// @brief Struct for the derivative or velocity based method
        struct {
            double tolerance;
        } Derivative;

        /// @brief Constructor for ExitConditions struct
        /// @param type the type of exit conditions used
        /// @param tolerance the error tolerance
        /// @param timeStep the cycle time of the loop
        /// @param requirement the derivative tolerance or required settle time
        ExitConditions(const Type type, const double tolerance, const int timeStep, const double requirement);

        /// @brief Updates and determines if the PID has settled
        /// @param error the PID's error
        /// @param derivative the current derivative value of the PID
        /// @return true if settled, false otherwise
        bool isSettled(const double error, const double derivative);
    };

private:

};

}