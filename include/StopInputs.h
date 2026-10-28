#pragma once
#include "RobotConfig.h"

namespace scara {
  class StopInputs {
  public:
    void begin();
    uint8_t pressedMask() const;  // Sofortige Auswertung, auch in der Timer-ISR.
    void poll();
    bool canReset() const;

  private:
    uint32_t lastPressedMs_ = 0;
    bool hadPress_ = false;
#ifdef __AVR__
    volatile uint8_t* ports_[2] = {nullptr, nullptr};
    uint8_t masks_[2] = {};
#endif
  };
}  // namespace scara
