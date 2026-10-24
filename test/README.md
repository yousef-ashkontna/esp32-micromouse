# Tests

Run `python scripts/test.py` from the repository root. Python 3 and a C++11 compiler are required. `g++` is used by default; set `CXX` to use another compiler.

- `host/navigation_test.cpp` checks route planning against an independent BFS.
- `host/firmware_test.cpp` exercises the firmware with simulated motors and sensors.
- `host/mocks/` supplies host-only replacements for Arduino and device interfaces.

Mocks are included only by the host build. They do not become part of the ESP32 firmware. Physical calibration and fault checks are described in [hardware acceptance](../docs/hardware.md#hardware-acceptance).
