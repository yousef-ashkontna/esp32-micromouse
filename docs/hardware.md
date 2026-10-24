# Hardware and calibration


`include/config.h` contains pin assignments, geometry, polarity, speed limits, and sensor thresholds. Distances are in millimeters, speeds in mm/s, and acceleration in mm/s².

| Setting | Value |
| --- | --- |
| Cell size | 180 mm |
| Wheel diameter | 65 mm |
| Wheel track, center to center | 85.5 mm |
| Encoder pulses per motor revolution per channel | 7 |
| Gear ratio | 1000:1 |
| Quadrature decoding | Four edges per cycle |
| Exploration / fast speed | 120 / 250 mm/s |
| Turn speed | 70 mm/s per wheel |
| PWM | 20 kHz, 10-bit |
| Front wall / stop threshold | 120 / 45 mm |
| IR wall threshold | 1700 on a 12-bit ADC |

The dimensions, gearing, and wiring retain the original project values. Motor and encoder polarity default to +1. Forward motion must produce positive counts on both wheels. Use the polarity settings to match the installed wiring. The speed-to-PWM feedforward value and proportional gains are initial settings, not measured motor characteristics.

| Connection | GPIO |
| --- | --- |
| Left motor PWM / direction | 32 / 18 |
| Right motor PWM / direction | 16 / 17 |
| Left encoder A / B | 19 / 33 |
| Right encoder A / B | 4 / 15 |
| Left IR near / far | 14 / 12 |
| Right IR near / far | 34 / 35 |
| I²C SDA / SCL | 21 / 22 |

The motor interface is PWM plus direction. Zero PWM stops the command; whether the driver coasts or electrically brakes depends on its hardware. The motion controller does not replace a hardware power cutoff.

## Hardware acceptance


Before deployment, verify one wheel revolution against encoder counts, a measured 180 mm move, and left/right 90-degree turns. Then verify wall detection at cell centers, stopping clearance at the configured speeds, a disconnected encoder, a disconnected front sensor, and Reset during travel. Run exploration, automatic return, and fast mode in a small known maze before using the full maze.

Compilation and host simulation cannot establish traction, real stopping distance, motor polarity, sensor placement, or battery-dependent behavior. Hardware acceptance remains required for a production release.
