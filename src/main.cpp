#include <Arduino.h>
#include <BluetoothSerial.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "maze.h"

namespace {
using namespace config;
using navigation::Maze;
using navigation::Path;
using navigation::Pose;

enum class State { Idle, Exploring, Returning, Ready, FastRun, Fault };
State state = State::Idle;
Maze maze;
Pose pose;
Path route;
VL53L0X tof;
BluetoothSerial bluetooth;
bool bluetoothReady = false, sensorReady = false, routeReady = false;
bool stopNeedsRelease = false;
uint16_t frontMm = 0;
uint32_t frontUpdatedAt = 0;
volatile int32_t encoderLeft = 0, encoderRight = 0;
portMUX_TYPE encoderLock = portMUX_INITIALIZER_UNLOCKED;

float limit(float value, float low, float high) {
    return fmaxf(low, fminf(value, high));
}

void report(const char *message) {
    Serial.println(message);
    if (bluetoothReady && bluetooth.hasClient()) bluetooth.println(message);
}

void stopMotors() {
    ledcWrite(leftChannel, 0);
    ledcWrite(rightChannel, 0);
}

bool fail(const char *reason) {
    stopMotors();
    state = State::Fault;
    routeReady = false;
    report(reason);
    return false;
}

void setMotors(int left, int right) {
    if (state == State::Fault || digitalRead(resetButton) == LOW) {
        stopMotors();
        return;
    }
    left *= leftMotorSign;
    right *= rightMotorSign;
    digitalWrite(leftDirectionPin, left >= 0 ? HIGH : LOW);
    digitalWrite(rightDirectionPin, right >= 0 ? HIGH : LOW);
    ledcWrite(leftChannel, static_cast<int>(limit(abs(left), 0, maxPwm)));
    ledcWrite(rightChannel, static_cast<int>(limit(abs(right), 0, maxPwm)));
}

void IRAM_ATTR leftAInterrupt() {
    const int step = digitalRead(leftEncoderA) == digitalRead(leftEncoderB) ? 1 : -1;
    portENTER_CRITICAL_ISR(&encoderLock);
    encoderLeft += step * leftEncoderSign;
    portEXIT_CRITICAL_ISR(&encoderLock);
}
void IRAM_ATTR leftBInterrupt() {
    const int step = digitalRead(leftEncoderA) != digitalRead(leftEncoderB) ? 1 : -1;
    portENTER_CRITICAL_ISR(&encoderLock);
    encoderLeft += step * leftEncoderSign;
    portEXIT_CRITICAL_ISR(&encoderLock);
}
void IRAM_ATTR rightAInterrupt() {
    const int step = digitalRead(rightEncoderA) == digitalRead(rightEncoderB) ? 1 : -1;
    portENTER_CRITICAL_ISR(&encoderLock);
    encoderRight += step * rightEncoderSign;
    portEXIT_CRITICAL_ISR(&encoderLock);
}
void IRAM_ATTR rightBInterrupt() {
    const int step = digitalRead(rightEncoderA) != digitalRead(rightEncoderB) ? 1 : -1;
    portENTER_CRITICAL_ISR(&encoderLock);
    encoderRight += step * rightEncoderSign;
    portEXIT_CRITICAL_ISR(&encoderLock);
}

struct WheelPosition { float left, right; };
WheelPosition wheelPosition() {
    portENTER_CRITICAL(&encoderLock);
    const int32_t left = encoderLeft, right = encoderRight;
    portEXIT_CRITICAL(&encoderLock);
    return {left / ticksPerMm, right / ticksPerMm};
}

void resetEncoders() {
    portENTER_CRITICAL(&encoderLock);
    encoderLeft = encoderRight = 0;
    portEXIT_CRITICAL(&encoderLock);
}

bool checkStop() {
    if (digitalRead(resetButton) == LOW) {
        stopNeedsRelease = true;
        return fail("Stopped. Place at start, then release and hold Reset.");
    }
    return state != State::Fault;
}

bool updateFront() {
    if (!sensorReady) return fail("Front sensor unavailable.");
    const uint8_t ready = tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (tof.last_status != 0) return fail("Front sensor communication failed.");
    if (ready & 7) {
        const uint16_t range = tof.readRangeContinuousMillimeters();
        if (tof.last_status != 0 || tof.timeoutOccurred() || range == 0 || range >= 8190)
            return fail("Front sensor returned an invalid range.");
        frontMm = range;
        frontUpdatedAt = millis();
    }
    if (millis() - frontUpdatedAt > sensorStaleMs) return fail("Front sensor data expired.");
    return checkStop();
}

bool freshFront() {
    const uint32_t started = millis();
    // Flush the last sample so a completed turn cannot reuse the old heading's range.
    if (!updateFront()) return false;
    const uint32_t previous = frontUpdatedAt;
    while (frontUpdatedAt == previous) {
        if (!updateFront()) return false;
        if (millis() - started > sensorStaleMs) return fail("Front sensor did not update.");
        delay(1);
    }
    return true;
}

bool initializeSensor() {
    sensorReady = false;
    tof.setTimeout(50);
    if (!tof.init() || !tof.setMeasurementTimingBudget(20000))
        return fail("Front sensor initialization failed.");
    tof.startContinuous();
    sensorReady = true;
    frontUpdatedAt = millis();
    return freshFront();
}

struct IrSample { int leftNear, leftFar, rightNear, rightFar; };
IrSample readIr() {
    IrSample sample = {};
    for (int i = 0; i < 4; ++i) {
        sample.leftNear += analogRead(leftNearIr);
        sample.leftFar += analogRead(leftFarIr);
        sample.rightNear += analogRead(rightNearIr);
        sample.rightFar += analogRead(rightFarIr);
    }
    sample.leftNear /= 4; sample.leftFar /= 4;
    sample.rightNear /= 4; sample.rightFar /= 4;
    return sample;
}

bool senseWalls() {
    if (!freshFront()) return false;
    const IrSample ir = readIr();
    maze.observe(pose.x, pose.y, pose.heading, frontMm < frontWallMm);
    maze.observe(pose.x, pose.y, (pose.heading + 3) & 3,
                 ir.leftNear > irWallThreshold || ir.leftFar > irWallThreshold);
    maze.observe(pose.x, pose.y, (pose.heading + 1) & 3,
                 ir.rightNear > irWallThreshold || ir.rightFar > irWallThreshold);
    return true;
}

float wallCorrection() {
    const IrSample ir = readIr();
    if (ir.leftNear <= irWallThreshold || ir.leftFar <= irWallThreshold ||
        ir.rightNear <= irWallThreshold || ir.rightFar <= irWallThreshold) return 0;
    return limit(wallKp * ((ir.leftNear + ir.leftFar) - (ir.rightNear + ir.rightFar)), -35, 35);
}

bool drive(float distance, float cruise, int leftSign, int rightSign, bool center) {
    if (distance <= 0 || cruise <= 0) return fail("Invalid motion request.");
    if (!checkStop() || !freshFront()) return false;
    const bool straight = leftSign == rightSign;
    resetEncoders();
    const uint32_t started = millis();
    uint32_t lastTick = started, leftProgressAt = started, rightProgressAt = started;
    const uint32_t timeout = static_cast<uint32_t>(distance / cruise * 4000.0f + 3000.0f);
    float lastLeft = 0, lastRight = 0, progressLeft = 0, progressRight = 0;
    float targetSpeed = 0;
    while (true) {
        if (!checkStop() || !updateFront()) return false;
        const uint32_t now = millis();
        if (now - started > timeout) return fail("Motion timed out.");
        if (now - lastTick < controlPeriodMs) { delay(1); continue; }
        const float dt = (now - lastTick) / 1000.0f;
        lastTick = now;
        const WheelPosition wheels = wheelPosition();
        const float left = wheels.left * leftSign, right = wheels.right * rightSign;
        const bool leftDone = left >= distance - positionToleranceMm;
        const bool rightDone = right >= distance - positionToleranceMm;
        if (left < -2 || right < -2) return fail("Encoder direction does not match motor direction.");
        if (fabsf(left - right) > maxWheelMismatchMm) return fail("Wheel tracking error.");
        if (leftDone && rightDone) { stopMotors(); delay(40); return checkStop(); }
        if (straight && frontMm <= frontStopMm) return fail("Obstacle interrupted motion. Position must be reset.");
        if (left >= progressLeft + 0.25f) { progressLeft = left; leftProgressAt = now; }
        if (right >= progressRight + 0.25f) { progressRight = right; rightProgressAt = now; }
        if ((!leftDone && now - leftProgressAt > stallTimeoutMs) ||
            (!rightDone && now - rightProgressAt > stallTimeoutMs)) return fail("Motor stalled or encoder disconnected.");
        const float remaining = fmaxf(0, distance - fmaxf(left, right));
        targetSpeed = fminf(cruise, fminf(targetSpeed + acceleration * dt,
                                        sqrtf(2.0f * acceleration * remaining)));
        const float leftSpeed = (left - lastLeft) / dt, rightSpeed = (right - lastRight) / dt;
        lastLeft = left; lastRight = right;
        const float feedforward = minPwm + (maxPwm - minPwm) * targetSpeed / speedAtFullPwm;
        float correction = headingKp * (left - right);
        if (center) correction -= wallCorrection();
        correction = limit(correction, -180, 180);
        const int leftPwm = leftDone ? 0 : static_cast<int>(limit(
            feedforward + speedKp * (targetSpeed - leftSpeed) - correction, minPwm, maxPwm));
        const int rightPwm = rightDone ? 0 : static_cast<int>(limit(
            feedforward + speedKp * (targetSpeed - rightSpeed) + correction, minPwm, maxPwm));
        setMotors(leftPwm * leftSign, rightPwm * rightSign);
    }
}

bool face(int heading) {
    if (heading < 0 || heading > 3) return fail("Invalid heading.");
    while (pose.heading != heading) {
        const int difference = (heading - pose.heading + 4) & 3;
        const bool right = difference != 3;
        if (!drive(quarterTurnMm, turnSpeed, right ? 1 : -1, right ? -1 : 1, false)) return false;
        pose.heading = (pose.heading + (right ? 1 : 3)) & 3;
    }
    return true;
}

bool advance(int count, bool fast) {
    Pose destination = pose;
    if (count <= 0 || count >= navigation::cells) return fail("Invalid straight segment.");
    for (int i = 0; i < count; ++i) {
        if (!maze.canTravel(destination.x, destination.y, pose.heading, true))
            return fail("Route contains an unverified passage.");
        destination.x += navigation::dx[pose.heading];
        destination.y += navigation::dy[pose.heading];
    }
    if (!drive(cellMm * count, fast ? fastSpeed : exploreSpeed, 1, 1, !fast)) return false;
    pose = destination;
    return true;
}

bool executeRoute(bool fast) {
    for (int i = 0; i < route.length;) {
        const int heading = route.directions[i];
        if (!face(heading)) return false;
        int count = 1;
        if (fast) {
            while (i + count < route.length && route.directions[i + count] == heading) ++count;
        }
        if (!advance(count, fast)) return false;
        i += count;
    }
    return true;
}

void exploreStep() {
    if (Maze::isGoal(pose.x, pose.y)) {
        report("Goal reached. Returning to start.");
        state = State::Returning;
        return;
    }
    if (!senseWalls()) return;
    maze.flood(false, false);
    const int direction = maze.nextDirection(pose, false);
    if (direction < 0) { fail("No route to the goal."); return; }
    if (!face(direction) || !freshFront()) return;
    if (frontMm < frontWallMm) {
        maze.observe(pose.x, pose.y, pose.heading, true);
        return;
    }
    maze.observe(pose.x, pose.y, pose.heading, false);
    advance(1, false);
}

class Button {
public:
    explicit Button(int pin) : pin_(pin) {}
    bool pressed() {
        const bool reading = digitalRead(pin_) == LOW;
        const uint32_t now = millis();
        if (reading != raw_) { raw_ = reading; changedAt_ = now; }
        if (now - changedAt_ < debounceMs || stable_ == raw_) return false;
        stable_ = raw_;
        return stable_;
    }
private:
    int pin_;
    bool raw_ = true, stable_ = true;
    uint32_t changedAt_ = 0;
};
Button exploreKey(exploreButton), fastKey(fastButton);
bool resetReleased = false, resetHeld = false, resetPending = false;
uint32_t resetStarted = 0;

bool handleReset() {
    if (digitalRead(resetButton) == HIGH) {
        stopNeedsRelease = false;
        resetReleased = true;
        resetHeld = false;
        if (resetPending) {
            resetPending = false;
            resetEncoders();
            maze.reset(); pose = Pose{}; route.length = 0; routeReady = false;
            state = State::Idle;
            if (initializeSensor()) report("Map cleared. Ready to explore from (0,0), facing north.");
        }
        return false;
    }
    stopMotors();
    if (stopNeedsRelease || !resetReleased) return true;
    if (!resetHeld) { resetHeld = true; resetStarted = millis(); }
    if (millis() - resetStarted >= resetHoldMs) resetPending = true;
    return true;
}
}

void setup() {
    pinMode(leftDirectionPin, OUTPUT); pinMode(rightDirectionPin, OUTPUT);
    pinMode(leftPwmPin, OUTPUT); pinMode(rightPwmPin, OUTPUT);
    digitalWrite(leftPwmPin, LOW); digitalWrite(rightPwmPin, LOW);
    pinMode(exploreButton, INPUT_PULLUP); pinMode(fastButton, INPUT_PULLUP);
    pinMode(resetButton, INPUT_PULLUP);
    ledcSetup(leftChannel, pwmFrequency, pwmBits);
    ledcSetup(rightChannel, pwmFrequency, pwmBits);
    ledcAttachPin(leftPwmPin, leftChannel); ledcAttachPin(rightPwmPin, rightChannel);
    stopMotors();
    Serial.begin(115200);
    bluetoothReady = bluetooth.begin("ESP32-MYM");
    if (!bluetoothReady) report("Bluetooth unavailable; USB logging remains active.");
    pinMode(leftEncoderA, INPUT); pinMode(leftEncoderB, INPUT);
    pinMode(rightEncoderA, INPUT); pinMode(rightEncoderB, INPUT);
    attachInterrupt(digitalPinToInterrupt(leftEncoderA), leftAInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftEncoderB), leftBInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightEncoderA), rightAInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightEncoderB), rightBInterrupt, CHANGE);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Wire.begin(sdaPin, sclPin);
    Wire.setTimeOut(20);
    maze.reset();
    if (initializeSensor()) report("Ready. Explore starts mapping; hold Reset for one second to clear.");
}

void loop() {
    if (handleReset()) { delay(5); return; }
    const bool explorePressed = exploreKey.pressed();
    const bool fastPressed = fastKey.pressed();
    if (state == State::Fault) { stopMotors(); delay(5); return; }
    if (explorePressed && (state == State::Idle || state == State::Ready)) {
        routeReady = false;
        state = State::Exploring;
        report("Exploration started.");
    } else if (fastPressed) {
        if (state == State::Ready && routeReady && pose.x == 0 && pose.y == 0 && pose.heading == 0) {
            state = State::FastRun;
            report("Fast run started.");
        } else report("Fast run requires a mapped route and a return to start.");
    }
    switch (state) {
    case State::Exploring:
        exploreStep();
        break;
    case State::Returning:
        if (!maze.buildPath(pose, true, route)) { fail("No verified route to start."); break; }
        if (!executeRoute(false) || !face(0)) break;
        if (pose.x != 0 || pose.y != 0 || !maze.buildPath(pose, false, route) || route.length == 0) {
            fail("Could not prepare the fast route."); break;
        }
        routeReady = true;
        state = State::Ready;
        report("At start. Fast run is ready.");
        break;
    case State::FastRun:
        if (!routeReady) { fail("Fast route is unavailable."); break; }
        if (!executeRoute(true)) break;
        if (!Maze::isGoal(pose.x, pose.y)) { fail("Fast route did not reach the goal."); break; }
        routeReady = false;
        state = State::Returning;
        report("Fast run complete. Returning to start.");
        break;
    default:
        stopMotors();
        break;
    }
    delay(5);
}
