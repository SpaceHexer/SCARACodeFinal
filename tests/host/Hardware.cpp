#include <Arduino.h>
#include "RobotConfig.h"
unsigned long testMicros = 0, edges[256] = {};
int pins[256] = {}, modeCalls[256] = {}, readCalls[256] = {}, writeCalls[256] = {};
bool drivenLow[256] = {}, drivenHigh[256] = {}, simulateSwitches = false;
int32_t physicalSteps[4] = {};
int txCapacity = 64;
FakeSerial Serial;
unsigned long micros() {
  return uint32_t(testMicros);
}
unsigned long millis() {
  return uint32_t(testMicros / 1000);
}
void pinMode(uint8_t pin, uint8_t mode) {
  ++modeCalls[pin];
  if (mode == INPUT_PULLUP)
    pins[pin] = HIGH;
}
int digitalRead(uint8_t pin) {
  ++readCalls[pin];
  if (drivenLow[pin])
    return LOW;
  if (drivenHigh[pin])
    return HIGH;
  if (simulateSwitches)
    for (uint8_t i = 0; i < 4; ++i)
      if (pin == scara::config::AXIS[i].limitPin)
        return physicalSteps[i] * scara::config::AXIS[i].homeDirection >= 100 ? LOW : HIGH;
  return pins[pin];
}
void digitalWrite(uint8_t pin, uint8_t value) {
  ++writeCalls[pin];
  if (value == HIGH && pins[pin] == LOW) {
    ++edges[pin];
    for (uint8_t i = 0; i < 4; ++i)
      if (pin == scara::config::AXIS[i].stepPin) {
        bool positive = pins[scara::config::AXIS[i].dirPin] == HIGH;
        physicalSteps[i] += (positive ? 1 : -1) * scara::config::AXIS[i].direction;
      }
  }
  pins[pin] = value;
}
