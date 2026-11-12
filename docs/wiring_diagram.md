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
| Battery+ through voltage divider (10K/2K) | A0 |

## Voltage Divider for Battery Monitoring
```
Battery+ ──[10K]──┬──[2K]── GND
                   │
                   └── A0 (Nano) / GPIO 36 (ESP32)

V_out = V_bat × 2 / (10 + 2) = V_bat / 6
For 12.6V (3S LiPo): V_out = 2.1V
```

The ratio has to match `BATTERY_DIVIDER_RATIO` in `config.h` (6.0). This
page used to show a 10K/4.7K divider, which disagreed with the code and puts
about 4 V on the pin: fine for the Nano's 5 V ADC, above the ESP32's 3.3 V.
10K/2K keeps a full pack inside both.

## ESP32 DevKit pins

The Nano pin numbers cannot be reused on an ESP32: GPIO 6 to 11 are wired to
the module's SPI flash. `config.h` selects these automatically for the
`esp32` and `esp32_hil` builds.

| Signal | ESP32 GPIO | Note |
|--------|-----------|------|
| Motor front left | 25 | LEDC PWM |
| Motor front right | 26 | LEDC PWM |
| Motor rear left | 27 | LEDC PWM |
| Motor rear right | 14 | LEDC PWM |
| RC CH1 roll | 34 | input only |
| RC CH2 pitch | 35 | input only |
| RC CH3 throttle | 32 | |
| RC CH4 yaw | 33 | |
| Battery divider | 36 (VP) | ADC1, readable with Wi-Fi on |
| MPU6050 SDA / SCL | 21 / 22 | default I2C |

The receiver channels are read with pin-change interrupts on both boards
(`receiver.h`), so any pin with interrupt support works; these avoid the
ESP32's boot strapping pins.

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
