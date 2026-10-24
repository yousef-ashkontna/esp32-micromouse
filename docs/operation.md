# Operation


Start at the center of cell (0, 0), facing north.

- **Explore (GPIO 27):** map a route to the center, return along observed passages, and face north at the start.
- **Fast run (GPIO 26):** follow the shortest route through the observed map, then return to the start. This button is enabled only after exploration and a successful return.
- **Reset (GPIO 25):** stops active motion immediately. To clear the map and position, place the robot back at the start, release Reset, hold it for one second, then release it again.

Buttons connect to ground and use internal pull-ups. Release the mode buttons before pressing them again. A fault disables motion until reset. Maps are held in RAM and cleared on restart.

USB serial runs at 115200 baud. Bluetooth reports the same status messages under `ESP32-MYM`; it does not accept drive commands.

## Navigation and fault handling


Exploration uses breadth-first flood distances and can plan through unknown passages. Each selected passage is checked before moving. Return and fast routes use only observed openings. Detected walls remain closed until reset. Fast straights combine consecutive cells; all turns rotate in place.

Position and heading change only after the corresponding movement completes. Interrupted movement latches a fault because the robot may be between cells. Both encoders are monitored for stalls, reversed polarity, excessive tracking error, and movement timeout. Front ranging is continuous, with communication, invalid-range, and stale-data checks during motion. Invalid ranges stop the robot rather than establishing an open passage.

The controller ramps target speed with acceleration and stopping-distance limits, then uses encoder speed feedback and wheel-distance correction. IR correction is applied during slow travel when both side walls are visible. Routes are shortest within the observed map; exploration does not prove a globally shortest path through every unexplored cell.
