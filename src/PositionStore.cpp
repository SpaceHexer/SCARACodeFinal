#include "PositionStore.h"
#include <math.h>

namespace scara {
  Error PositionStore::save(uint8_t id, const Pose& p) {
    if (id >= CAPACITY)
      return Error::BadArgs;
    Real values[] = {p.x, p.y, p.z, p.psi};
    for (Real v : values)
      if (!isfinite(v))
        return Error::BadArgs;
    poses_[id] = p;
    used_ |= uint16_t(1) << id;
    return Error::None;
  }

  Error PositionStore::load(uint8_t id, Pose& p) const {
    if (id >= CAPACITY)
      return Error::BadArgs;
    if (!occupied(id))
      return Error::EmptySlot;
    p = poses_[id];
    return Error::None;
  }

  Error PositionStore::erase(uint8_t id) {
    if (id >= CAPACITY)
      return Error::BadArgs;
    used_ &= uint16_t(~(uint16_t(1) << id));
    return Error::None;
  }
}  // namespace scara
