# IoT-Based Temperature Feedback Fan Control System

## Overview

This project implements a **distributed embedded system** for controlling fan speed in a cold storage or cafe inventory setting based on real-time temperature feedback.

The system provides a practical, modular, and responsive alternative to commercial smart cooling systems.

---

## Hardware Architecture

### Components

* Arduino Nano 33 IoT: For environmental sensing and network communication.
* DHT11 Temperature Sensor: Low-cost sensor for Celsius readings.
* SSD1306 OLED Display (I2C, 128x64): For displaying real-time temperature.
* Raspberry Pi 3B+: Acts as the control node for running a PID-controlled feedback loop and as a server for setting desired temperatures.
* 5V PWM Controllable Cooling Fan: Actuated through Pi's GPIO18 (PWM0).
* Wi-Fi Network: Enables wireless data transmission between Arduino and Raspberry Pi.

![Hardware Setup](assets/hardware%20setup%20-%20raspberry%20pi%20and%20arduino.jpg)

### Arduino Wiring

| Component  | Arduino Pin |
| ---------- | ----------- |
| DHT11 VCC  | 3.3V        |
| DHT11 GND  | GND         |
| DHT11 DATA | D2          |
| OLED VCC   | 3.3V        |
| OLED GND   | GND         |
| OLED SDA   | A4 (SDA)    |
| OLED SCL   | A5 (SCL)    |

### Raspberry Pi Fan Setup

* GPIO18 (PWM0) is connected to the fan's PWM control line.
* VCC and GND lines from the fan connect to the Pi's 5V and GND respectively.
* Ensure `/boot/config.txt` has PWM enabled or run:

  ```bash
  sudo dtoverlay pwm-2chan
  ```

---

## Software Architecture

### Arduino (`arduino_sensor/temp_oled.ino`)

* Initializes the DHT11 and OLED libraries.
* Displays temperature on-screen using Adafruit's SSD1306.
* Sends temperature to Raspberry Pi every 2 seconds via HTTP POST.

Example payload:

```json
{"temperature": 26.40}
```

### Raspberry Pi (`rasp_server/fan_control.cpp`)

* Uses `httplib.h` to run a lightweight HTTP server.
* Accepts temperature data via `/temp` and updates PID loop.
* Computes PWM duty cycle and writes to `/sys/class/pwm/...` to control fan.
* Also supports `/setpoint` POST to allow real-time temperature tuning.
* `/` route serves an HTML dashboard using `ui.html`.

### Frontend Dashboard (`rasp_server/ui.html`)

* Displays live temperature graph with setpoint.
* Input box for updating target temperature.

![Dashboard](assets/fan%20control%20dashboard%20on%20server\(graph\).jpg)

---

## PID Control

### What is PID?

A **PID controller** (Proportional-Integral-Derivative) is an automatic control algorithm.

* **Proportional**: Responds to current error.
* **Integral**: Reacts to accumulated past errors.
* **Derivative**: Predicts future error trend.

## PWM and Duty Cycle

**Pulse Width Modulation (PWM)** is a method of simulating analog control using digital signals. It switches power on and off rapidly, adjusting the amount of **"on time"** within each cycle to simulate a continuous voltage level.

### **Period**

The total time of one full PWM cycle (e.g., **40,000 nanoseconds = 25 kHz**)

### **Duty Cycle**

The proportion of the period where the signal is **HIGH** (i.e., power **ON**)

---

### **Example**

If the period is **40,000 ns (25 kHz)**:

* **10,000 ns HIGH time** means **25% duty** → low fan speed  
* **20,000 ns HIGH time** means **50% duty** → medium fan speed  
* **40,000 ns HIGH time** means **100% duty** → full fan speed  

The Raspberry Pi adjusts this in real time based on **PID output**.

---

The implementation includes:

* Setpoint: Desired temperature
* Input: Current temperature
* Output: PWM duty cycle (0–100%) to adjust fan speed
* Clamping, anti-windup, and deadband enhancements for stability

## PID Controller bringing real time temperature closer to desired setpoint

![PID Control](assets/pid%20control%20towards%20destired%20temperature%20\(graph\).jpg)

---

## Fault Tolerance Features

* **Deadband**: Prevents over-reaction near setpoint.  
  If the temperature fluctuates within a small range around the setpoint, the system avoids unnecessary fan speed adjustments, ensuring stability and reducing wear on the fan.

* **Integral clamping**: Avoids excessive buildup during saturation.  
  This prevents the PID controller from overcompensating when the fan reaches its maximum or minimum speed, ensuring smoother recovery when conditions change.

* **Graceful shutdown**: Fan control stops Raspberry Pi Arduino crashes.  
  The fan's PWM signal is disabled, ensuring the fan is detached from Raspberry PI control(simulated). If the Arduino is still operational, it can continue sensing and displaying temperature on the OLED. Additionally, a fallback mechanism simulates the fan running at a default 50% duty cycle, mimicking manufacturer-provided cooling.

* **Signal Handlers**: Ensures clean fan disable on SIGINT or termination.  
  When the Raspberry Pi program is terminated, the fan is stopped cleanly to prevent it from running indefinitely. If the Arduino is still active, it can continue sending temperature data, which can be logged or used by another system.

---

## How it Works (End-to-End)

1. **Sensing**: Arduino reads temperature and shows it on OLED.
2. **Communication**: Arduino sends JSON temperature to Raspberry Pi.
3. **Control**: Pi computes PID and adjusts fan PWM output.
4. **Monitoring**: Web dashboard reflects live values and allows setpoint tuning.

---

## Setup Instructions

### On Arduino

1. Connect Arduino Nano 33 IoT via USB
2. Open `temp_oled.ino` in Arduino IDE
3. Upload and open Serial Monitor
4. OLED will display temperature, and HTTP POST will be sent every 2s

## Arduino side connection

![Setpoint Updated](assets/arduino%20connection(cli).png)

### On Raspberry Pi

* **`httplib.h`**: A lightweight C++ HTTP library used to implement the HTTP server on the Raspberry Pi. It handles incoming requests (e.g., temperature data) and serves the dashboard.
* **`json.hpp`**: A single-header JSON library for parsing and generating JSON data. It is used to process temperature data sent from the Arduino and to handle setpoint updates from the dashboard.

These libraries are essential for enabling communication between the Raspberry Pi, Arduino, and the web dashboard.

```bash
# Install required libraries
sudo apt update
sudo apt install g++ libfmt-dev

# Download required header files
wget https://raw.githubusercontent.com/yhirose/cpp-httplib/master/httplib.h -O rasp_server/httplib.h
wget https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp -O rasp_server/json.hpp

# Clone and compile
cd rasp_server
sudo dtoverlay pwm-2chan

# Compile
g++ -std=c++17 fan_control.cpp -o fan_control -lfmt -lpthread

# Run
sudo ./fan_control
```

## Starting Raspberry PI Controller

![Server Startup](assets/starting%20raspberry%20pi%20server\(cli\).png)

## Setpoint updated on dashboard

![Setpoint Updated](assets/setpoint%20updated%20from%20dashboard\(cli\).png)

## Deadband Enhancement in action when we reach desired setpoint

![Fan Duty to 0](assets/fan%20duty%20down%20to%200%20when%20setpoint%20reached\(cli\).png)

## Graph

![Duty Dropping](assets/fan%20duty%20dropping%20to%200%20once%20setpoint%20reached\(graph\).jpg)

---

## Final Output Summary

* OLED and Serial log real-time temperature.
* Fan automatically adapts speed to reach and maintain the setpoint.
* Dashboard shows trends and allows dynamic configuration.
* All components are modular, responsive, and fault-resilient.

Ideal for:

* Cold storage automation
* Cafes, restaurants, bakeries
* Laboratories or indoor horticulture setups

---

## New Feature: Predefined Modes for Easy Control

The system now includes **three predefined modes** for controlling the fan's behavior. These modes allow users to optimize the system for their specific needs without requiring in-depth technical knowledge of PID tuning.

### Available Modes

1. **Balanced Mode (Default)**:

   * Provides a middle ground between energy efficiency and performance.
   * Ideal for environments where moderate cooling is sufficient, such as small cafes, offices, or home setups.
   * Ensures stable temperature control with minimal fan speed fluctuations.

2. **Energy Saving Mode**:
   * Prioritizes energy efficiency by reducing fan speed and responsiveness.
   * Suitable for scenarios where cooling demand is low, such as during off peak hours or in energy conscious environments.
   * Helps reduce power consumption while maintaining acceptable temperature levels.

3. **Performance Mode**:
   * Maximizes responsiveness and cooling performance.
   * Designed for high demand scenarios, such as peak hours in cafes, cold storage, or laboratories.
   * Ensures rapid temperature stabilization at the cost of higher energy usage.

### How It Works

* Users can switch between these modes using the web dashboard with a single click.
* The system automatically adjusts the PID controller's parameters (Kp, Ki, Kd) based on the selected mode.
* No technical knowledge of PID tuning is required—just select the mode that best fits your needs.

### Reference Output

Below is a screenshot showing the system operating in Balanced Mode, maintaining a stable temperature with minimal fan speed adjustments:

![Balanced Mode Output](assets/energy%20saving%20and%20best%20performance%20mode.png)

These predefined modes make the system versatile and user-friendly, catering to a wide range of use cases without requiring manual configuration.
