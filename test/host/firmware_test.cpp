#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include "../../src/main.cpp"

SerialPort Serial;
WirePort Wire;
uint32_t clockMs = 0, stopAt = UINT_MAX;
int outputs[40] = {}, buttons[40] = {}, pwm[2] = {};
bool stalledLeft = false, stalledRight = false, reversed = false;
double accumulatedLeft = 0, accumulatedRight = 0;
uint32_t millis() { return clockMs; }
int digitalRead(int pin) { return pin == resetButton && clockMs >= stopAt ? LOW : buttons[pin]; }
void digitalWrite(int pin, int value) { outputs[pin] = value; }
int analogRead(int) { return 500; }
void ledcWrite(int channel, int value) { pwm[channel] = value; }
void delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
        ++clockMs;
        auto travel = [](int duty) { return duty == 0 ? 0.0 : (duty - minPwm) * speedAtFullPwm / (maxPwm - minPwm) + 10.0; };
        double left = stalledLeft ? 0 : travel(pwm[0]) * (outputs[leftDirectionPin] ? 1 : -1);
        double right = stalledRight ? 0 : travel(pwm[1]) * (outputs[rightDirectionPin] ? 1 : -1);
        if (reversed) left = -left;
        accumulatedLeft += left * ticksPerMm / 1000;
        accumulatedRight += right * ticksPerMm / 1000;
        int lt = static_cast<int>(accumulatedLeft), rt = static_cast<int>(accumulatedRight);
        encoderLeft += lt; encoderRight += rt;
        accumulatedLeft -= lt; accumulatedRight -= rt;
    }
}
void resetFixture() {
    clockMs = 0; stopAt = UINT_MAX;
    for (int &button : buttons) button = HIGH;
    pwm[0] = pwm[1] = 0;
    stalledLeft = stalledRight = reversed = false;
    accumulatedLeft = accumulatedRight = 0;
    resetEncoders();
    maze.reset(); pose = Pose{}; route.length = 0;
    state = State::Exploring; routeReady = false;
    tof = VL53L0X{}; sensorReady = true; frontMm = 500; frontUpdatedAt = 0;
    resetReleased = resetHeld = resetPending = stopNeedsRelease = false;
}
void assertStopped() { assert(pwm[0] == 0 && pwm[1] == 0); }
int main() {
    resetFixture();
    maze.observe(0, 0, 0, false);
    assert(advance(1, false));
    assert(pose.x == 0 && pose.y == 1);
    assertStopped();
    assert(face(1) && pose.heading == 1);
    assert(encoderLeft > 0 && encoderRight < 0);
    assert(face(0) && pose.heading == 0);
    assert(encoderLeft < 0 && encoderRight > 0);
    assert(face(2) && pose.heading == 2);
    assertStopped();

    resetFixture(); maze.observe(0, 0, 0, false); stopAt = 300;
    assert(!advance(1, false) && state == State::Fault && pose.y == 0); assertStopped();
    handleReset(); delay(resetHoldMs + 10); handleReset();
    assert(state == State::Fault && !resetPending);
    stopAt = UINT_MAX; handleReset();
    assert(state == State::Fault && !stopNeedsRelease);
    resetFixture(); maze.observe(0, 0, 0, false); tof.range = 30;
    assert(!advance(1, false) && pose.y == 0); assertStopped();
    resetFixture(); stalledLeft = stalledRight = true;
    assert(!drive(cellMm, exploreSpeed, 1, 1, false) && clockMs < 1500); assertStopped();
    resetFixture(); stalledRight = true;
    assert(!drive(cellMm, exploreSpeed, 1, 1, false)); assertStopped();
    resetFixture(); reversed = true;
    assert(!drive(cellMm, exploreSpeed, 1, 1, false)); assertStopped();
    resetFixture(); tof.stalled = true;
    assert(!drive(cellMm, exploreSpeed, 1, 1, false) && clockMs < 200); assertStopped();
    resetFixture(); tof.range = 8190;
    assert(!freshFront() && state == State::Fault); assertStopped();
    resetFixture(); tof.last_status = 2;
    assert(!freshFront() && state == State::Fault); assertStopped();
    resetFixture(); tof.healthy = false;
    assert(!initializeSensor() && state == State::Fault); assertStopped();
    resetFixture();
    assert(!advance(1, true) && pose.y == 0); assertStopped();

    resetFixture();
    for (int y = 0; y < 7; ++y) maze.observe(0, y, 0, false);
    for (int x = 0; x < 7; ++x) maze.observe(x, 7, 1, false);
    pose.x = pose.y = 7; state = State::Returning;
    loop();
    assert(state == State::Ready && routeReady && pose.x == 0 && pose.y == 0 && pose.heading == 0);
    assert(route.length == 14); assertStopped();
    state = State::FastRun; loop();
    assert(state == State::Returning && Maze::isGoal(pose.x, pose.y)); assertStopped();

    resetFixture(); state = State::Fault; routeReady = true;
    handleReset(); buttons[resetButton] = LOW; handleReset();
    delay(resetHoldMs); handleReset(); buttons[resetButton] = HIGH; handleReset();
    assert(state == State::Idle && !routeReady && pose.x == 0 && pose.y == 0);
    assertStopped();
    puts("Firmware simulation passed: motion, turns, emergency stop, obstacles, stalls, encoder polarity, sensor faults, return, fast run, reset.");
}
