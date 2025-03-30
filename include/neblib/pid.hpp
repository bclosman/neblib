#pragma once

#include "neblib/util/util.hpp"

#include <cmath>

namespace neblib {

/// @brief PID Controller
class PID {
public:

    /// @brief Gains (P, I, D) for PID object
    struct Gains {
        double kP;
        double kI;
        double kD;
        bool resetIntegralOnPass;
        double windupRange;


        /// @brief Constructor for Gains struct
        /// @param kP proportional constant
        /// @param kI integral constant
        /// @param kD derivative constant 
        /// @param windupRange range where integration takes place
        Gains(const double kP, const double kI, const double kD, const bool resetIntegralOnPass, const double windupRange = INFINITY);
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
    Gains gains;
    ExitConditions exitConditions;

    double integral = 0;
    double previousError = 0;
    bool settled = false;

public:
    /// @brief Creates a PID object
    /// @param gains PID gains
    /// @param exitConditions PID exit conditions
    PID(const Gains& gains, const ExitConditions& exitConditions);

    /// @brief Creates a PID object
    /// @param gains PID gains
    /// @param exitConditions PID exit conditions
    PID(Gains&& gains, ExitConditions&& exitConditions);


    /// @brief Returns the PID output
    /// @param error error or distance from target
    double getOutput(const double error);

    /// @brief Returns the PID output
    /// @param error error or distance from target
    /// @param maxOutput the maximum acceptable output
    /// @param minOutput the minimum acceptable output
    double getOutput(const double error, const double maxOutput, const double minOutput);

    /// @brief Returns whether or not the PID has settled
    bool isSettled();
};

}