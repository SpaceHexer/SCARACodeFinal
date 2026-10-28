#pragma once
#include "Types.h"

namespace scara {
  struct Sample {
    Real position, velocity, acceleration, jerk;
  };

  class QuinticProfile {
  public:
    bool configure(Real T_fahrt, Real q0, Real v0, Real a0, Real q1, Real v1, Real a1);
    Sample evaluate(Real t_lauf) const;
    // Prüft alle inneren Extremstellen mittels rekursiver Polynom-Nullstellen.
    Error check(const AxisConfig& limits) const;

    const Real* coefficients() const {
      return b_;
    }

  private:
    Real b_[6] = {}, duration_ = 0;
    Real endPosition_ = 0, endVelocity_ = 0, endAcceleration_ = 0;
  };
}  // namespace scara
