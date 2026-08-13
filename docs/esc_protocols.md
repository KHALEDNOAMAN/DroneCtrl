# ESC Communication Protocols

| Protocol | Type | Resolution | Refresh |
|----------|------|------------|--------|
| Standard PWM | Analog | 1000us | 50Hz |
| OneShot125 | Analog | 125us | 400Hz |
| OneShot42 | Analog | 42us | 1.2kHz |
| DShot150 | Digital | 11-bit | 8kHz |
| DShot300 | Digital | 11-bit | 16kHz |
| DShot600 | Digital | 11-bit | 32kHz |

## Why DShot?
- No calibration needed
- CRC error checking
- Bidirectional telemetry
- Jitter-free digital signal