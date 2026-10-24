#pragma once
#include "Arduino.h"
class VL53L0X {
public:
    enum { RESULT_INTERRUPT_STATUS = 0x13 };
    uint8_t last_status = 0;
    uint16_t range = 500;
    uint32_t sampledAt = 0;
    bool healthy = true, stalled = false;
    void setTimeout(int) {}
    bool init() { return healthy; }
    bool setMeasurementTimingBudget(int) { return healthy; }
    void startContinuous() {}
    uint8_t readReg(int) { return !stalled && millis() - sampledAt >= 20 ? 7 : 0; }
    uint16_t readRangeContinuousMillimeters() { sampledAt = millis(); return range; }
    bool timeoutOccurred() { return false; }
};
