#pragma once
#include <stdint.h>

namespace config {
constexpr int leftPwmPin = 32, leftDirectionPin = 18;
constexpr int rightPwmPin = 16, rightDirectionPin = 17;
constexpr int leftEncoderA = 19, leftEncoderB = 33;
constexpr int rightEncoderA = 4, rightEncoderB = 15;
constexpr int leftNearIr = 14, leftFarIr = 12;
constexpr int rightNearIr = 34, rightFarIr = 35;
constexpr int exploreButton = 27, fastButton = 26, resetButton = 25;
constexpr int sdaPin = 21, sclPin = 22;
constexpr int leftMotorSign = 1, rightMotorSign = 1;
constexpr int leftEncoderSign = 1, rightEncoderSign = 1;
constexpr float pi = 3.14159265358979323846f;
constexpr float cellMm = 180.0f, wheelDiameterMm = 65.0f, trackWidthMm = 85.5f;
constexpr int encoderPulses = 7;
constexpr float gearRatio = 1000.0f;
constexpr float ticksPerMm = encoderPulses * 4.0f * gearRatio / (pi * wheelDiameterMm);
constexpr float quarterTurnMm = pi * trackWidthMm / 4.0f;
constexpr int pwmFrequency = 20000, pwmBits = 10;
constexpr int leftChannel = 0, rightChannel = 1;
constexpr int maxPwm = (1 << pwmBits) - 1, minPwm = 220;
constexpr float exploreSpeed = 120.0f, fastSpeed = 250.0f, turnSpeed = 70.0f;
constexpr float acceleration = 350.0f, speedAtFullPwm = 300.0f;
constexpr float speedKp = 1.2f, headingKp = 12.0f, wallKp = 0.008f;
constexpr float positionToleranceMm = 0.5f, maxWheelMismatchMm = 18.0f;
constexpr int irWallThreshold = 1700, frontWallMm = 120, frontStopMm = 45;
constexpr uint32_t controlPeriodMs = 10, sensorStaleMs = 120;
constexpr uint32_t stallTimeoutMs = 800, debounceMs = 35, resetHoldMs = 1000;
static_assert(ticksPerMm > 0 && cellMm > 0 && trackWidthMm > 0, "Invalid drive geometry");
static_assert(exploreSpeed > 0 && fastSpeed >= exploreSpeed && acceleration > 0,
              "Invalid motion profile");
static_assert(leftMotorSign * leftMotorSign == 1 && rightMotorSign * rightMotorSign == 1 &&
              leftEncoderSign * leftEncoderSign == 1 && rightEncoderSign * rightEncoderSign == 1,
              "Drive polarity must be +1 or -1");
}
