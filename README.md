<div align="center">

# 🚁 DroneCtrl

**Quadcopter Flight Controller with PID Stabilization, Sensor Fusion & Interactive 3D Simulator.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](#)
[![Language](https://img.shields.io/badge/language-C++_|_TypeScript-blue.svg)](#)
[![Simulator](https://img.shields.io/badge/simulator-Three.js-orange.svg)](#)

</div>

DroneCtrl is an open-source flight control system that bridges the gap between hardware engineering and software simulation. It provides a robust C++ firmware for ESP32/Arduino-based quadcopters, featuring real-time PID stabilization and sensor fusion. Alongside the firmware, it includes an interactive 3D web simulator to test algorithms and tune parameters safely before real-world flight.

## Overview

DroneCtrl is a complete quadcopter flight control system designed for learning and experimentation. It combines real ESP32/Arduino firmware with an interactive 3D web simulator, allowing you to understand drone stabilization without needing physical hardware.

The system implements industry-standard techniques: PID control loops for attitude stabilization, complementary filter for sensor fusion, and motor mixing algorithms for translating desired movements into individual motor speeds.

---

## ✨ Key Features

- 🧠 **Real ESP32/Arduino firmware** for quadcopter control
- ⚖️ **PID stabilization** (pitch, roll, yaw)
- 📡 **MPU6050 IMU sensor fusion** (complementary filter)
- 🗺️ **GPS waypoint navigation**
- 🛡️ **Failsafe modes** (signal loss, low battery, geofence)
- 🎮 **Interactive 3D web simulator** (Three.js)
- 📊 **Telemetry dashboard** with real-time gauges
- 🎛️ **Configurable PID tuning interface**

## 🏗️ Architecture

```text
Sensors (IMU/GPS/Baro) → Sensor Fusion → PID Controller → Motor Mixer → ESC → Motors
                                           ↑
                                  Setpoint from RC/Waypoints
```

## 🛠️ Tech Stack

| Layer | Technology |
| :--- | :--- |
| **Firmware** | C++, Arduino, ESP32, PlatformIO |
| **Sensors** | MPU6050 (IMU), BMP280 (Barometer), GPS |
| **Control** | PID loops, Complementary filter, Motor mixing |
| **Simulator** | TypeScript, Three.js, WebGL |
| **Dashboard** | React, Chart.js, WebSocket |

## ⚙️ How It Works

### Flight Controller Loop
The core firmware runs a deterministic control loop at 400Hz. This ensures minimal latency between reading sensor data and applying corrective forces to the motors.

### Sensor Fusion
Raw IMU data is notoriously noisy. We use a complementary filter to combine the fast response of the gyroscope with the stable, long-term accuracy of the accelerometer, yielding precise attitude estimation.

### PID Stabilization
Three independent PID (Proportional-Integral-Derivative) controllers calculate the required correction for Pitch, Roll, and Yaw based on the difference between the desired setpoint and the current estimated attitude.

### Motor Mixing
The quadcopter uses an X-configuration. The motor mixer translates the aggregate Pitch, Roll, Yaw, and Throttle commands into specific PWM signals for each of the four Electronic Speed Controllers (ESCs).

## 🚀 Getting Started

### Prerequisites
- Node.js (v16+)
- PlatformIO IDE

### Hardware Setup
1. Connect the ESP32 to the MPU6050 IMU via I2C.
2. Wire the 4 ESC signal lines to the designated ESP32 PWM pins.
3. Ensure proper power distribution and grounding.

### Firmware Flash
1. Open the project in PlatformIO.
2. Connect your ESP32 via USB.
3. Build and upload the firmware.

### Simulator
To run the local 3D web simulator and telemetry dashboard:
```bash
npm install
npm run dev
```

## 📂 Project Structure

```text
DroneCtrl/
├── firmware/          # ESP32 C++ Flight Controller Code
│   ├── src/           # Main logic, PID, Sensor Fusion
│   ├── include/       # Headers, Config
│   └── platformio.ini # Build configuration
├── simulator/         # Three.js 3D Web Simulator
│   ├── src/           # TypeScript source
│   └── public/        # Assets, 3D models
├── dashboard/         # React Telemetry Interface
└── docs/              # Documentation and wiring diagrams
```

## 🎛️ PID Tuning Guide

Tuning is critical for stable flight. Start with these steps:
1. **P (Proportional)**: Increase until the drone oscillates rapidly, then reduce by 20-30%. This provides the immediate corrective force.
2. **D (Derivative)**: Increase to dampen the P-term oscillations and soften the response to rapid changes. Too much D causes jitter.
3. **I (Integral)**: Increase slowly to hold attitude against external forces (like wind or off-center CG) over time.

## 🕹️ Simulator Controls

| Action | Control |
| :--- | :--- |
| **Throttle Up/Down** | `W` / `S` |
| **Yaw Left/Right** | `A` / `D` |
| **Pitch Forward/Back** | `Arrow Up` / `Arrow Down` |
| **Roll Left/Right** | `Arrow Left` / `Arrow Right` |
| **Camera Orbit** | `Mouse Drag` |
| **Reset Simulation** | `R` |

## 🗺️ Roadmap

- [x] Basic stabilization
- [x] 3D simulator
- [x] GPS waypoints
- [ ] Optical flow integration
- [ ] Return to home (RTH) failsafe
- [ ] FPV camera feed streaming

## 🤝 
---

## Screenshots & Demo

### 3D Simulator
```
┌─────────────────────────────────────────────┐
│  DroneCtrl - 3D Flight Simulator            │
│  ┌───────────────────────────────────────┐  │
│  │                                       │  │
│  │        🚁  Quadcopter View            │  │
│  │      Altitude: 15.2m                  │  │
│  │      Roll: 2.3°  Pitch: -1.1°        │  │
│  │      Yaw: 45.0°                       │  │
│  │                                       │  │
│  └───────────────────────────────────────┘  │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────────┐  │
│  │Thrust│ │ Roll │ │Pitch │ │   Yaw    │  │
│  │ 65%  │ │ 2.3° │ │-1.1° │ │  45.0°   │  │
│  │ ████ │ │ ██   │ │ ██   │ │ ████     │  │
│  └──────┘ └──────┘ └──────┘ └──────────┘  │
└─────────────────────────────────────────────┘
```

### Telemetry Dashboard
- Real-time attitude gauges (roll, pitch, yaw)
- Motor speed indicators (M1-M4)
- PID tuning sliders with live response
- Altitude and GPS position display
- Battery voltage monitoring

### Live Demo
> Run the simulator locally: `npm install && npm run dev`
> Open `http://localhost:3000` in your browser
> Use WASD keys to control the drone!


Contributing
Contributions are welcome! Please feel free to submit a Pull Request. Make sure to read our contributing guidelines before getting started.

## 📜 License
This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments
- Inspired by MultiWii and Betaflight.
- Three.js community for excellent WebGL resources.
