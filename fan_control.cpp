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

// PWM Control Namespace
namespace pwm
{
    constexpr std::string_view kExportPath{"/sys/class/pwm/pwmchip0/export"};
    constexpr std::string_view kEnablePath{"/sys/class/pwm/pwmchip0/pwm0/enable"};
    constexpr std::string_view kPeriodPath{"/sys/class/pwm/pwmchip0/pwm0/period"};
    constexpr std::string_view kDutyPath{"/sys/class/pwm/pwmchip0/pwm0/duty_cycle"};

    void write(std::string_view path, std::string_view value)
    {
        std::ofstream file{path.data()};
        if (!file)
        {
            throw std::runtime_error(fmt::format("Failed to open: {}", path));
        }
        file << value;
    }

    void exportChannel()
    {
        write(kExportPath, "0");
    }

    void enable(bool on)
    {
        write(kEnablePath, on ? "1" : "0");
    }

    void setPeriod(uint32_t ns)
    {
        write(kPeriodPath, std::to_string(ns));
    }

    void setDutyCycle(uint32_t ns)
    {
        write(kDutyPath, std::to_string(ns));
    }

    void setup()
    {
        exportChannel();
        std::this_thread::sleep_for(std::chrono::milliseconds{100}); // Ensure pwm0 is ready
        setPeriod(40000);                                            // 25kHz
        setDutyCycle(0);                                             // Start with 0% duty
        enable(true);
    }
}

// Time helper for PID
double secondsSince(const std::chrono::steady_clock::time_point &start)
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - start).count();
}

int main()
{
    constexpr uint32_t kPwmPeriod = 40000;
    pwm::setup();

    // PID initialized with Kp, Ki, Kd and output clamp [0, 100]
    PID<double> pid{6.0, 0.5, 1.0, 0.0, 100.0};
    pid.setSetpoint(26.0); // Target temperature in Celsius

    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();

    httplib::Server server;

    server.Post("/temp", [&pid, &lastTime](const httplib::Request &req, httplib::Response &res)
                {
        try {
            auto json = nlohmann::json::parse(req.body);
            double temperature = json.at("temperature");

            double dt = secondsSince(lastTime);
            lastTime = std::chrono::steady_clock::now();

            double pidOutput = pid.compute(temperature, dt);
            pidOutput = std::clamp(pidOutput, 0.0, 100.0); // Clamp for fan safety

            uint32_t dutyNs = static_cast<uint32_t>((pidOutput / 100.0) * kPwmPeriod);
            pwm::setDutyCycle(dutyNs);

            fmt::print("Received Temp: {:.2f} °C | PID Out: {:.2f} | Fan Duty: {:.0f}%\n",
                       temperature, pidOutput, (pidOutput));

            res.set_content("OK", "text/plain");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(fmt::format("Error: {}", e.what()), "text/plain");
        } });

    std::cout << "HTTP Server running on port 8000...\n";
    server.listen("0.0.0.0", 8000);
}
