#pragma once
#include "Types.h"

namespace scara {
  class PositionStore {
  public:
    static constexpr uint8_t CAPACITY = 32;
    void begin();
    Error save(uint8_t id, const Pose& pose);
    Error load(uint8_t id, Pose& pose) const;
    Error erase(uint8_t id);

    bool occupied(uint8_t id) const {
      return id < CAPACITY && (used_ & (uint32_t(1) << id));
    }

  private:
    Pose poses_[CAPACITY];
    uint32_t used_ = 0;
    static_assert(CAPACITY <= sizeof(used_) * 8, "Belegungsmaske ist zu klein");
  };
}  // namespace scara
