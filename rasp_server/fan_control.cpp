#include <iostream>
#include <fstream>
#include <string_view>
#include <cstdint>
#include <stdexcept>
#include <chrono>
#include "json.hpp"
#include "httplib.h"
#include <fmt/core.h>
#include "pid.hpp"
#include <csignal> // For signal handling
#include <sstream> // For serving HTML content

// PWM Control Namespace
namespace pwm
{
    // Paths to control the PWM hardware on the system
    constexpr std::string_view kExportPath{"/sys/class/pwm/pwmchip0/export"};        // Path to enable PWM channel
    constexpr std::string_view kEnablePath{"/sys/class/pwm/pwmchip0/pwm0/enable"};   // Path to enable/disable PWM
    constexpr std::string_view kPeriodPath{"/sys/class/pwm/pwmchip0/pwm0/period"};   // Path to set PWM period
    constexpr std::string_view kDutyPath{"/sys/class/pwm/pwmchip0/pwm0/duty_cycle"}; // Path to set duty cycle

    /**
     * @brief Writes a value to a file at the specified path.
     *
     * This is used to control the PWM hardware by writing configuration values
     * (e.g., enable, period, duty cycle) to the appropriate system files.
     *
     * @param path The file path to write to.
     * @param value The value to write.
     */
    void write(std::string_view path, std::string_view value)
    {
        std::ofstream file{path.data()};
        if (!file)
        {
            throw std::runtime_error(fmt::format("Failed to open: {}", path));
        }
        file << value;
    }

    /**
     * @brief Exports the PWM channel for use.
     *
     * This enables the PWM channel (e.g., pwm0) so it can be configured and used.
     */
    inline void exportChannel()
    {
        write(kExportPath, "0");
    }

    /**
     * @brief Enables or disables the PWM output.
     *
     * @param on If true, enables the PWM output; otherwise, disables it.
     */
    inline void enable(bool on)
    {
        write(kEnablePath, on ? "1" : "0");
    }

    /**
     * @brief Sets the PWM period in nanoseconds.
     * What is PWM?
     * PWM is a technique used to control devices like motors, LEDs, or fans by rapidly turning
     * them on and off. The frequency of the signal determines how fast this on/off cycle happens.
     *
     * The period determines how fast the signal repeats its on/off cycle.
     * - A shorter period means the signal repeats faster (higher frequency).
     * - A longer period means the signal repeats slower (lower frequency).
     *
     * Real-world analogy:
     * Imagine turning a light on and off repeatedly:
     * - Period: The total time for one on/off cycle.
     * - Frequency: How many cycles happen in one second (1 / period).
     *
     * For example:
     * - A period of 40000 ns (40 microseconds) corresponds to 25 kHz (25,000 cycles per second).
     * - A period of 100000 ns (100 microseconds) corresponds to 10 kHz (10,000 cycles per second).
     *
     * @param ns The period in nanoseconds.
     */
    inline void setPeriod(const uint32_t ns)
    {
        write(kPeriodPath, std::to_string(ns));
    }

    /**
     * @brief Sets the PWM duty cycle in nanoseconds.
     *
     * The duty cycle controls how long the signal stays "on" during one cycle.
     * - A higher duty cycle means the fan runs faster because it stays "on" longer.
     * - A lower duty cycle means the fan runs slower because it stays "on" for a shorter time.
     * - A duty cycle of 0% means the fan is always "off."
     * - A duty cycle of 100% means the fan is always "on."
     *
     * For example:
     * - If the period is 40000 ns (25 kHz) and the duty cycle is 20000 ns, the fan is "on" for 50% of the time.
     *
     * @param ns The duty cycle in nanoseconds.
     */
    inline void setDutyCycle(const uint32_t ns)
    {
        write(kDutyPath, std::to_string(ns));
    }

    /**
     * @brief Sets up the PWM channel with default settings.
     *
     * - Exports the PWM channel.
     * - Sets the period to 40000 ns (25 kHz).
     * - Sets the duty cycle to 0 (0% duty).
     * - Enables the PWM output.
     */
    void setup()
    {
        exportChannel();
        std::this_thread::sleep_for(std::chrono::milliseconds{100}); // Ensure pwm0 is ready
        setPeriod(40000);                                            // 25kHz
        setDutyCycle(0);                                             // Start with 0% duty
        enable(true);
    }
}

// Namespace for cleanup utilities
namespace cleanup_util
{
    /**
     * @brief Signal handler to clean up and stop the fan.
     *
     * This function is called when the program receives a termination signal (e.g., SIGINT, SIGTERM, etc.).
     */
    void handleSignal(int signal)
    {
        fmt::print("\nStopping fan and cleaning up...\n");
        pwm::setDutyCycle(0); // Set duty cycle to 0% to stop the fan
        pwm::enable(false);   // Disable the PWM output
        std::exit(0);         // Exit the program
    }

    /**
     * @brief Registers the signal handler for termination signals.
     *
     * This ensures the cleanup logic is executed for signals like SIGINT, SIGTERM, etc.
     */
    void registerSignalHandlers()
    {
        std::signal(SIGINT, handleSignal);  // Handle Ctrl+C
        std::signal(SIGTERM, handleSignal); // Handle termination signals
    }
}

// Namespace for timer utilities
namespace timer_util
{
    /**
     * @brief Calculates the time elapsed since a given start time.
     *
     * @param start The start time point.
     * @return The elapsed time in seconds as a double.
     */
    double secondsSince(const std::chrono::steady_clock::time_point &start)
    {
        using namespace std::chrono;
        return duration_cast<duration<double>>(steady_clock::now() - start).count();
    }
}

// Namespace for server utilities
namespace server_util
{
    float latestTemperature = 0.0f; // Store the most recent temperature value

    /**
     * @brief Starts the HTTP server to handle requests.
     *
     * - Serves the HTML page for the user interface.
     * - Handles `/setpoint` POST requests to update the PID setpoint.
     * - Handles `/temp` POST requests to process temperature data.
     * - Provides `/latest-temp` endpoint to serve the most recent temperature value.
     *
     * @param pid Reference to the PID controller.
     * @param kPwmPeriod PWM period in nanoseconds.
     */
    void startServer(PID &pid, const float kPwmPeriod)
    {
        httplib::Server server;
        std::chrono::steady_clock::time_point lastTime{std::chrono::steady_clock::now()};

        // Serve the HTML page for the user interface
        server.Get("/", [](const httplib::Request &, httplib::Response &res)
                   {
            std::ifstream htmlFile("ui.html"); // Use relative path
            if (htmlFile)
            {
                std::stringstream buffer;
                buffer << htmlFile.rdbuf();
                res.set_content(buffer.str(), "text/html");
            }
            else
            {
                res.status = 500;
                res.set_content("Error: Unable to load UI page.", "text/plain");
            } });

        // Endpoint to update the PID setpoint
        server.Post("/setpoint", [&pid](const httplib::Request &req, httplib::Response &res)
                    {
            try
            {
                auto json = nlohmann::json::parse(req.body);
                float newSetpoint{json.at("setpoint").get<float>()}; // Parse new setpoint
                pid.setSetpoint(newSetpoint);

                fmt::print("Setpoint updated to: {:.2f} °C\n", newSetpoint);
                res.set_content("Setpoint updated successfully.", "text/plain");
            }
            catch (const std::exception &e)
            {
                res.status = 400;
                res.set_content(fmt::format("Error: {}", e.what()), "text/plain");
            } });

        // Endpoint to handle temperature data and compute PID output
        server.Post("/temp", [&pid, &lastTime, kPwmPeriod](const httplib::Request &req, httplib::Response &res)
                    {
            try
            {
                auto json = nlohmann::json::parse(req.body);
                float temperature{json.at("temperature").get<float>()}; // Explicitly parse as float

                float dt{static_cast<float>(timer_util::secondsSince(lastTime))}; // Convert elapsed time to float
                lastTime = std::chrono::steady_clock::now();

                // Compute the PID output
                float pidOutput{pid.compute(temperature, dt)};

                // Convert PID output (0–100%) to duty cycle in nanoseconds
                uint32_t dutyNs{static_cast<uint32_t>((pidOutput / 100.0f) * kPwmPeriod)};
                pwm::setDutyCycle(dutyNs);

                // Update the latest temperature value
                latestTemperature = temperature;

                fmt::print("Received Temp: {:.2f} °C | PID Out: {:.2f} | Fan Duty: {:.0f}%\n",
                           temperature, pidOutput, pidOutput);

                res.set_content("OK", "text/plain");
            }
            catch (const std::exception &e)
            {
                res.status = 400;
                res.set_content(fmt::format("Error: {}", e.what()), "text/plain");
            } });

        // Endpoint to get the latest temperature value
        server.Get("/latest-temp", [](const httplib::Request &, httplib::Response &res)
                   { res.set_content(fmt::format("{{\"temperature\": {:.2f}}}", latestTemperature), "application/json"); });

        fmt::print("HTTP Server running on port 8000...\n");
        server.listen("0.0.0.0", 8000);
    }
}

int main()
{
    // Register signal handlers for cleanup
    cleanup_util::registerSignalHandlers();

    constexpr float kPwmPeriod{40000.0f}; // PWM period in nanoseconds (25 kHz) as float
    pwm::setup();

    // PID initialized with Kp, Ki, Kd and output clamp [0, 100]
    PID pid{4.0f, 0.3f, 0.8f, 0.0f, 100.0f}; // Adjusted gains for better response
    pid.setSetpoint(24.0f);                  // Default target temperature in Celsius

    // Start the server
    server_util::startServer(pid, kPwmPeriod);

    // Cleanup in case the server stops unexpectedly
    pwm::setDutyCycle(0);
    pwm::enable(false);
}
