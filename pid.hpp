#pragma once

#include <algorithm> // for std::clamp
#include <cstdint>   // for fixed-size types like uint32_t

/**
 * @brief A generic, header-only PID controller.
 *
 * This class allows precision control for processes like temperature,
 * speed, or pressure using a Proportional-Integral-Derivative loop.
 *
 * The time delta (`dt`) must be supplied manually on each call to compute(),
 * allowing flexible integration across platforms — including embedded systems.
 */
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
    constexpr PID(float kp, float ki, float kd, float min_out, float max_out) noexcept
        : Kp{kp}, Ki{ki}, Kd{kd},
          minOutput{min_out}, maxOutput{max_out}
    {
    }

    /**
     * @brief Sets the target value the PID should drive toward.
     *
     * For example, for a fan cooling system, this could be 26.0°C.
     *
     * @param sp Desired setpoint
     */
    constexpr inline void setSetpoint(float sp) noexcept
    {
        setpoint = sp;
    }

    /**
     * @brief Returns the current setpoint for inspection or logging.
     */
    [[nodiscard]] constexpr inline float getSetpoint() const noexcept
    {
        return setpoint;
    }

    /**
     * @brief Performs a PID computation step.
     *
     * - Calculates the error as the difference between the input and the setpoint.
     * - The integral term accumulates the error over time to address persistent errors.
     * - The derivative term reacts to how quickly the error is changing.
     * - The output is clamped (restricted) between `minOutput` and `maxOutput` to ensure it stays within bounds.
     * - Anti-windup prevents the integral term from growing when the output is already at its limit.
     *
     * @param input Current process value (e.g., temperature in °C)
     * @param dt Time delta since the last update (in seconds)
     * @return Control output (e.g., 0–100% fan speed)
     */
    [[nodiscard]] float compute(float input, float dt) noexcept
    {
        // Calculate the difference between the current value and the target value
        const float error{input - setpoint};

        // Update the integral term by adding the error over time
        float newIntegral{integral + error * dt};

        // Calculate the rate of change of the error (derivative term)
        const float derivative{(dt > 0) ? (error - prevError) / dt : 0.0f};

        // Compute the PID output using the proportional, integral, and derivative terms
        const float output{Kp * error + Ki * newIntegral + Kd * derivative};

        // If the output is within the allowed range, update the integral term
        if (output >= minOutput && output <= maxOutput)
        {
            integral = newIntegral;
        }

        // Save the current error for the next derivative calculation
        prevError = error;

        // Restrict the output to the specified range
        return std::clamp(output, minOutput, maxOutput);
    }

private:
    // PID gain constants
    float Kp{0.0f}, Ki{0.0f}, Kd{0.0f};

    // Output limits
    float minOutput{0.0f}, maxOutput{0.0f};

    // Target value
    float setpoint{0.0f};

    // Internal state
    float prevError{0.0f}; // Last error for derivative calculation
    float integral{0.0f};  // Accumulated integral term
};
