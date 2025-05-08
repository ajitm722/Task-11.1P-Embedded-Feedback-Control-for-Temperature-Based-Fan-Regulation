#pragma once

#include <algorithm> // for std::clamp
#include <cstdint>   // for fixed-size types like uint32_t
#include <cmath>     // for std::abs

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
     * the function can be updated to validate the setpoint value
     * @param sp Desired setpoint
     */
    constexpr inline void setSetpoint(const float sp) noexcept
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
     * - The proportional term reacts to the current error.
     * - The integral term accumulates the error over time to address persistent errors.
     * - The derivative term reacts to how quickly the error is changing.
     * - The output is clamped (restricted) between `minOutput` and `maxOutput` to ensure it stays within bounds.
     * - Anti-windup prevents the integral term from growing when the output is already at its limit.
     * - Introduces a deadband around the setpoint to force output to 0 when the error is small.
     *
     * @param input Current process value (e.g., temperature in °C)
     * @param dt Time delta since the last update (in seconds)
     * @return Control output (e.g., 0–100% fan speed)
     */
    [[nodiscard]] float compute(float input, float dt) noexcept
    {
        // Calculate the difference between the current value and the target value
        const float error{input - setpoint};

        // Introduce a deadband around the setpoint
        constexpr float deadband = 0.05f; // Adjust this value as needed
        if (std::abs(error) <= deadband)
        {
            return 0.0f; // Force output to 0 within the deadband
        }

        // Proportional term: Reacts to the current error
        const float proportional = Kp * error;

        // Integral term: Accumulates the error over time to address persistent errors
        float newIntegral{integral + error * dt};

        // Clamp the integral term to prevent windup
        constexpr float integralClamp = 50.0f; // Adjust this value as needed
        newIntegral = std::clamp(newIntegral, -integralClamp, integralClamp);

        // Derivative term: Reacts to the rate of change of the error
        const float rawDerivative = (dt > 0) ? (error - prevError) / dt : 0.0f;

        // Clamp the derivative term to prevent spikes
        constexpr float derivativeClamp = 10.0f; // Adjust this value as needed
        const float derivative = std::clamp(rawDerivative, -derivativeClamp, derivativeClamp) * Kd;

        // Compute the PID output using the proportional, integral, and derivative terms
        const float output = proportional + Ki * newIntegral + derivative;

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
