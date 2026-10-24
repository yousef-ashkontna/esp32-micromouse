# ESP32 Micromouse

Encoder-based micromouse firmware for an ESP32, two N20 motors, four analog IR sensors, and a VL53L0X front distance sensor. It explores a 16 x 16 maze, reaches one of the four center cells, returns to the start, and runs the shortest route through the observed map.

## Features

- Breadth-first flood planning with reciprocal walls and explicit passage discovery.
- Encoder feedback, acceleration limits, in-place turns, and combined fast straights.
- Automatic return to the start before fast mode is enabled.
- Motion timeouts, encoder stall detection, front-sensor fault handling, and a reset interlock.
- USB and Bluetooth status reporting.
- Host regression tests and an ESP32 build workflow.

## Project layout

```text
.github/workflows/ci.yml    Automated tests and firmware build
src/main.cpp               Hardware control and run-state machine
include/config.h           Pins, geometry, thresholds, and motion settings
include/maze.h             Maze representation and route planning
test/host/                 Navigation and simulated firmware tests
scripts/test.py            Portable host-test runner
docs/hardware.md           Wiring, calibration, and acceptance checks
docs/operation.md          Buttons, navigation, and fault recovery
platformio.ini             Pinned ESP32 build configuration
```

## Build

Install Python and PlatformIO Core, then build from the repository root:

```sh
python -m pip install platformio==6.1.18
python -m platformio run -e esp32dev
```

The build uses Espressif32 6.12.0, Arduino ESP32 2.0.17, and Pololu VL53L0X 1.3.1. Build output is written to `.pio/build/esp32dev/`.

After checking the wiring and calibration settings, upload and open the serial monitor:

```sh
python -m platformio run -e esp32dev --target upload
python -m platformio device monitor --baud 115200
```

## Run

Place the robot at the center of cell (0, 0), facing north. Press **Explore** to map a route and return to the start, then press **Fast run**. **Reset** stops active motion immediately; hold it for one second and release it to clear the map after placing the robot back at the start.

See [operation](docs/operation.md) for the complete button behavior and fault recovery, and [hardware and calibration](docs/hardware.md) for wiring and physical settings.

## Test

With Python and `g++` on PATH:

```sh
python scripts/test.py
```

On Windows, the existing PowerShell entry point is also available:

```powershell
./test/host/run.ps1
```

The navigation suite checks boundaries, reciprocal walls, disconnected goals, all four center cells, a 255-step corridor, and 51,200 randomized route queries against an independent BFS. Firmware simulation covers travel, turns, obstacles, sensor failures, encoder faults, emergency stop, return-to-start, fast mode, and reset.

GitHub Actions runs the host tests and builds the ESP32 firmware for pushes and pull requests. Successful builds provide firmware binaries as a workflow artifact.

## Validation status

The ESP32 release build and host tests pass. The original drivetrain dimensions and wiring are retained in `include/config.h`; speed and controller settings require calibration on the assembled robot. Hardware acceptance is still required before deployment.
