#pragma once
#include "RobotConfig.h"

namespace scara {
  class Kinematics {
  public:
    Pose forward(const Joints& q) const;
    Error inverse(const Pose& goal, const Joints& seed, Joints& result) const;
  };
}  // namespace scara
