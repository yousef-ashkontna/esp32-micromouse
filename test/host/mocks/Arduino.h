#pragma once
#include <stdint.h>
#include <stdlib.h>
#define IRAM_ATTR
constexpr int HIGH = 1, LOW = 0, OUTPUT = 1, INPUT = 0, INPUT_PULLUP = 2, CHANGE = 3, ADC_11db = 3;
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL(portMUX_TYPE *) {}
inline void portENTER_CRITICAL_ISR(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE *) {}
uint32_t millis();
void delay(uint32_t ms);
int digitalRead(int pin);
void digitalWrite(int pin, int value);
int analogRead(int pin);
void ledcWrite(int channel, int value);
inline void pinMode(int, int) {}
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void analogReadResolution(int) {}
inline void analogSetAttenuation(int) {}
struct SerialPort { void begin(int) {} void println(const char *) {} };
extern SerialPort Serial;
