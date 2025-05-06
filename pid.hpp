#pragma once

#include <algorithm> // for std::clamp
#include <cstdint>   // for fixed-size types like uint32_t

/**
 * @brief A generic, header-only PID controller.
 *
 * This template class allows precision control for processes like temperature,
 * speed, or pressure using a Proportional-Integral-Derivative loop.
 *
 * The time delta (`dt`) must be supplied manually on each call to compute(),
 * allowing flexible integration across platforms — including embedded systems.
 *
 * @tparam T Floating-point type (e.g., float or double)
 */
template <typename T = double>
class PID
{
public:
    /**
     * @brief Constructs a new PID controller with gains and output limits.
     *
     * @param kp Proportional gain — reacts to current error
     * @param ki Integral gain     — reacts to accumulated error over time
     * @param kd Derivative gain   — reacts to rate of error change
     * @param min_out Lower bound on controller output
     * @param max_out Upper bound on controller output
     */
    PID(T kp, T ki, T kd, T min_out, T max_out)
        : Kp{kp}, Ki{ki}, Kd{kd},
          minOutput{min_out}, maxOutput{max_out},
          prevError{0.0}, integral{0.0}, setpoint{0.0}
    {
    }

    /**
     * @brief Sets the target value the PID should drive toward.
     *
     * For a fan cooling system, this could be 26.0°C, for example.
     *
     * @param sp Desired setpoint
     */
    void setSetpoint(T sp)
    {
        setpoint = sp;
    }

    /**
     * @brief Returns the current setpoint for inspection/logging.
     */
    T getSetpoint() const
    {
        return setpoint;
    }

    /**
     * @brief Performs a PID computation step.
     *
     * Uses:
     * - Error = input - setpoint (i.e., temperature - target)
     *   → Positive error means we're above target and must cool more
     *
     * - Integral term accumulates over time to correct persistent error
     * - Derivative term damps sudden changes
     *
     * The final output is clamped between [minOutput, maxOutput].
     *
     * @param input Current process value (e.g., temperature in °C)
     * @param dt Time delta since last update (in seconds)
     * @return Control output (e.g., 0–100% fan speed)
     */
    T compute(T input, T dt)
    {
        const T error = input - setpoint;

        // Compute tentative integral
        T newIntegral = integral + error * dt;

        // Compute proportional, derivative
        const T derivative = (dt > 0) ? (error - prevError) / dt : 0.0;
        const T output = Kp * error + Ki * newIntegral + Kd * derivative;

        // Clamp and apply anti-windup: only update integral if output is within bounds
        if (output >= minOutput && output <= maxOutput)
        {
            integral = newIntegral;
        }

        prevError = error;

        // Clamp final output
        return std::clamp(output, minOutput, maxOutput);
    }

private:
    // PID gain constants
    T Kp, Ki, Kd;

    // Output limits
    T minOutput, maxOutput;

    // Target value
    T setpoint;

    // Internal state
    T prevError; // Last error for derivative calculation
    T integral;  // Accumulated integral term
};
