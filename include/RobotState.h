#pragma once
#include "Types.h"

namespace scara {
  // Bits 0..3 kennzeichnen J1/J2/J3/Z separat; keine globale falsche Home-Freigabe.
  class RobotState {
  public:
    void markReferenced(uint8_t mask) {
      referenced_ |= mask;
    }

    void invalidate(uint8_t mask = 0x0f) {
      referenced_ &= uint8_t(~mask);
    }

    bool referenced(uint8_t mask) const {
      return (referenced_ & mask) == mask;
    }

    uint8_t referenceMask() const {
      return referenced_;
    }

  private:
    uint8_t referenced_ = 0;
  };
}  // namespace scara
