#pragma once
#include "Types.h"

namespace scara {
  class PositionStore {
  public:
    static constexpr uint8_t CAPACITY = 16;
    void begin();
    Error save(uint8_t id, const Pose& pose);
    Error load(uint8_t id, Pose& pose) const;
    Error erase(uint8_t id);

    bool occupied(uint8_t id) const {
      return id < CAPACITY && (used_ & (uint16_t(1) << id));
    }

  private:
    Pose poses_[CAPACITY];
    uint16_t used_ = 0;
  };
}  // namespace scara
