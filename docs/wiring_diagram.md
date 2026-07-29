# Wiring Diagram — DroneCtrl Flight Controller

## Quadcopter X-Configuration

```
        FRONT
    M1(CCW)   M2(CW)
      \       /
       \     /
        [FC]        ← Flight Controller (Arduino Nano)
       /     \
      /       \
    M3(CW)   M4(CCW)
        REAR
```

## Pin Connections

### Motor ESCs (PWM Output)
| Motor | Position | Direction | Arduino Pin | ESC Signal Wire |
|-------|----------|-----------|-------------|----------------|
| M1 | Front-Left | CCW | D3 | Yellow |
| M2 | Front-Right | CW | D5 | Yellow |
| M3 | Rear-Left | CW | D6 | Yellow |
| M4 | Rear-Right | CCW | D9 | Yellow |

### MPU6050 IMU (I2C)
| MPU6050 Pin | Arduino Pin |
|-------------|-------------|
| VCC | 5V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |
| INT | D10 (optional) |

### RC Receiver (PPM Input)
| Channel | Function | Arduino Pin |
|---------|----------|-------------|
| CH1 | Roll | D2 |
| CH2 | Pitch | D4 |
| CH3 | Throttle | D7 |
| CH4 | Yaw | D8 |

### Battery Monitor
| Connection | Arduino Pin |
|-----------|-------------|
| Battery+ through voltage divider (10K/4.7K) | A0 |

## Voltage Divider for Battery Monitoring
```
Battery+ ──[10K]──┬──[4.7K]── GND
                   │
                   └── A0 (Arduino)

V_out = V_bat × 4.7 / (10 + 4.7) = V_bat × 0.32
For 12.6V (3S LiPo): V_out = 4.03V (safe for 5V ADC)
```

## Parts List (BOM)

| # | Component | Model/Spec | Qty | Approx Cost |
|---|-----------|-----------|-----|-------------|
| 1 | Flight Controller | Arduino Nano | 1 | $5 |
| 2 | IMU Sensor | MPU6050 | 1 | $3 |
| 3 | Barometer | BMP280 | 1 | $3 |
| 4 | ESC | 30A Brushless | 4 | $24 |
| 5 | Motor | 2212 1000KV | 4 | $28 |
| 6 | Propeller | 10x4.5 | 4+4 | $6 |
| 7 | Frame | 450mm Quad | 1 | $15 |
| 8 | Battery | 3S 2200mAh LiPo | 1 | $18 |
| 9 | RC Transmitter | FlySky FS-i6 | 1 | $45 |
| 10 | RC Receiver | FS-iA6B | 1 | (included) |
| | | | **Total** | **~$147** |
