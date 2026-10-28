#include "StopInputs.h"
#include <Arduino.h>

namespace scara {
  void StopInputs::begin() {
    // Deaktivierte Taster: weder pinMode noch digitalRead/Registerzugriff.
    if (config::STOP_BUTTON_1_ENABLED) {
      pinMode(config::STOP_BUTTON_1_PIN, INPUT_PULLUP);
#ifdef __AVR__
      ports_[0] = portInputRegister(digitalPinToPort(config::STOP_BUTTON_1_PIN));
      masks_[0] = digitalPinToBitMask(config::STOP_BUTTON_1_PIN);
#endif
    }
    if (config::STOP_BUTTON_2_ENABLED) {
      pinMode(config::STOP_BUTTON_2_PIN, INPUT_PULLUP);
#ifdef __AVR__
      ports_[1] = portInputRegister(digitalPinToPort(config::STOP_BUTTON_2_PIN));
      masks_[1] = digitalPinToBitMask(config::STOP_BUTTON_2_PIN);
#endif
    }
    lastPressedMs_ = millis();
    hadPress_ = pressedMask() != 0;
  }

  uint8_t StopInputs::pressedMask() const {
    uint8_t result = 0;
    if (config::STOP_BUTTON_1_ENABLED) {
#ifdef __AVR__
      if ((*ports_[0] & masks_[0]) == 0)
        result |= 1;
#else
      if (digitalRead(config::STOP_BUTTON_1_PIN) == LOW)
        result |= 1;
#endif
    }
    if (config::STOP_BUTTON_2_ENABLED) {
#ifdef __AVR__
      if ((*ports_[1] & masks_[1]) == 0)
        result |= 2;
#else
      if (digitalRead(config::STOP_BUTTON_2_PIN) == LOW)
        result |= 2;
#endif
    }
    return result;
  }

  void StopInputs::poll() {
    if (pressedMask()) {
      lastPressedMs_ = millis();
      hadPress_ = true;
    }
  }

  bool StopInputs::canReset() const {
    // Nur eine tatsächlich beobachtete Betätigung braucht Freigabe-Entprellung.
    // Ohne aktivierte Taster gibt es weder GPIO-Zugriff noch Phantom-Sperrzeit.
    return pressedMask() == 0 && (!hadPress_ || uint32_t(millis() - lastPressedMs_) >= config::RELEASE_STABLE_MS);
  }
}  // namespace scara
