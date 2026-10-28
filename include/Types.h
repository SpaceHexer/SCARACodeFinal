#pragma once
#include <stdint.h>

namespace scara {
  using Real = float;  // Absichtlich auch im Hosttest AVR-gleiche 32-Bit-Präzision.
  constexpr uint8_t AXES = 4;
  enum class Axis : uint8_t { J1, J2, J3, Z };

  struct Joints {
    Real q[4] = {};
  };  // rad, rad, rad, mm.

  struct Pose {
    Real x = 0, y = 0, z = 0, psi = 0;
  };

  struct Boundary {
    Joints position, velocity, acceleration;
  };

  struct AxisConfig {
    uint8_t stepPin, dirPin, enablePin, limitPin;
    Real stepsPerUnit, qRef;  // n_ref=0, q=q_ref+n/K.
    int8_t direction;
    Real minimum, maximum, maxSpeed, maxAcceleration, maxJerk;
    int8_t homeDirection;
    uint16_t seekSpeed, latchSpeed, clearanceSteps;
    int32_t homeOffsetSteps;
  };
  enum class Error : uint8_t { None, BadArgs, Busy, NotHomed, Stopped, ButtonHeld, NotConfigured, Unreachable, Degenerate, JointLimit, ProfileLimit, Duration, Continuity, NeedSuccessor, Underrun, Timing, StepRate, Endstop, HomeTimeout, Storage, EmptySlot };
  const char* errorName(Error e);

  inline uint8_t axisBit(uint8_t axis) {
    return uint8_t(1U << axis);
  }
}  // namespace scara
