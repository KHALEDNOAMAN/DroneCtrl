<div align="center">

# 🚁 DroneCtrl

**Quadcopter Flight Controller with PID Stabilization, Sensor Fusion & Interactive 3D Simulator**

[![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![React](https://img.shields.io/badge/React-61DAFB?style=for-the-badge&logo=react&logoColor=black)](https://react.dev/)
[![Three.js](https://img.shields.io/badge/Three.js-000000?style=for-the-badge&logo=three.js&logoColor=white)](https://threejs.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-3178C6?style=for-the-badge&logo=typescript&logoColor=white)](https://www.typescriptlang.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](LICENSE)

**Real embedded firmware** for Arduino/ESP32 + **Playable 3D simulator** to test control algorithms before flight.

[🎮 **Try the Simulator**](https://drone-ctrl-elnoaman.vercel.app) | [📄 **Firmware Docs**](docs/wiring_diagram.md)

</div>

---

## 🏗️ Architecture — Software-in-the-Loop (SIL)

```
┌─────────────────────────────────────────────────┐
│                  DroneCtrl                       │
├──────────────────┬──────────────────────────────┤
│   /firmware      │     /simulator               │
│   Real C++ Code  │     Web Game (Three.js)      │
│                  │                              │
│  ┌────────────┐  │  ┌─────────────────────────┐│
│  │ PID Control│◄─┼──│  Same PID Algorithm     ││
│  │ Algorithm  │  │  │  (TypeScript port)      ││
│  └────────────┘  │  └─────────────────────────┘│
│  ┌────────────┐  │  ┌─────────────────────────┐│
│  │ MPU6050    │  │  │  Simulated Gyro+Accel   ││
│  │ Sensors    │  │  │  with noise model       ││
│  └────────────┘  │  └─────────────────────────┘│
│  ┌────────────┐  │  ┌─────────────────────────┐│
│  │ 4x ESC    │  │  │  Simulated Motors       ││
│  │ PWM Output │  │  │  with thrust model      ││
│  └────────────┘  │  └─────────────────────────┘│
├──────────────────┴──────────────────────────────┤
│  Test in simulator → Flash to real hardware     │
└─────────────────────────────────────────────────┘
```

---

## 🎮 Simulator Controls

| Key | Action |
|-----|--------|
| W / S | Pitch forward / back |
| A / D | Roll left / right |
| ← / → | Yaw left / right |
| Space | Throttle up |
| Shift | Throttle down |
| H | Toggle auto-hover |
| R | Reset position |
| T | Toggle telemetry panel |
| P | Toggle PID tuning sliders |
| 1-3 | Camera views (chase / top / FPV) |

---

## 🔧 Firmware — Real Hardware

### Components Needed

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | Arduino Nano / ESP32 | Main flight controller |
| IMU Sensor | MPU6050 | Gyroscope + Accelerometer |
| Barometer | BMP280 | Altitude measurement |
| ESCs | 4x 30A | Motor speed control |
| Motors | 4x 2212 1000KV Brushless | Thrust |
| Frame | 450mm Quadcopter | Structure |
| Battery | 3S 2200mAh LiPo | Power |
| Receiver | FlySky FS-iA6B | Remote control input |

### Pin Mapping

```
Arduino Nano:
├── D3  → ESC Motor 1 (Front-Left)  [PWM]
├── D5  → ESC Motor 2 (Front-Right) [PWM]
├── D6  → ESC Motor 3 (Rear-Left)   [PWM]
├── D9  → ESC Motor 4 (Rear-Right)  [PWM]
├── A4  → MPU6050 SDA              [I2C]
├── A5  → MPU6050 SCL              [I2C]
├── D2  → Receiver CH1 (Roll)      [PPM]
├── D4  → Receiver CH2 (Pitch)     [PPM]
├── D7  → Receiver CH3 (Throttle)  [PPM]
├── D8  → Receiver CH4 (Yaw)       [PPM]
└── A0  → Battery Voltage           [ADC]
```

---

## 🚀 Quick Start

### Simulator (no hardware needed)
```bash
cd simulator
npm install
npm run dev
# Open http://localhost:5173 — fly the drone! 🚁
```

### Firmware (with hardware)
```bash
# Install PlatformIO CLI
pip install platformio

cd firmware
pio run              # Build
pio run -t upload    # Flash to Arduino
pio device monitor   # Serial monitor
```

---

## 📊 Control Theory

### PID Controller
```
Error = Target - Current
P = Kp × Error
I = Ki × ∫Error dt
D = Kd × dError/dt
Output = P + I + D
```

### Motor Mixing (X-config)
```
Motor1 (FL) = Throttle + PitchPID + RollPID - YawPID
Motor2 (FR) = Throttle + PitchPID - RollPID + YawPID
Motor3 (RL) = Throttle - PitchPID + RollPID + YawPID
Motor4 (RR) = Throttle - PitchPID - RollPID - YawPID
```

---

## 📝 License
MIT License — see [LICENSE](LICENSE) file.

---
<div align="center">

Built by [Khaled Noaman](https://github.com/KHALEDNOAMAN) — Computer Engineering Student 🚀

</div>
