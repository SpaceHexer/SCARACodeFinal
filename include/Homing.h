#pragma once
#include "MotorController.h"
#include "RobotState.h"
namespace scara {
class Homing {
 public:
  enum State : uint8_t { Idle, Running, Done, Failed };
  Homing(MotorController& m, RobotState& s) : motors_(m), robot_(s) {}
  Error start(uint8_t mask, bool sequential);
  void update();
  void cancel();
  bool busy() const {
    return state_ == Running;
  }
  State state() const {
    return state_;
  }
  Error error() const {
    return error_;
  }

 private:
  enum Phase : uint8_t {
    Off,
    Preclear,
    FastSeek,
    Release,
    SlowSeek,
    ClearFast,
    ClearSlow,
    Pause,
    Offset,
    Finish
  };
  struct AxisHome {
    Phase phase = Off, next = Off;
    uint32_t started = 0, stableSince = 0, pauseSince = 0;
    int32_t lastCount = 0, total = 0;
    bool observed = false;
  };
  AxisHome axes_[4];
  MotorController& motors_;
  RobotState& robot_;
  State state_ = Idle;
  Error error_ = Error::None;
  uint8_t pending_ = 0, active_ = 0;
  bool sequential_ = false;
  Error launchGroup();
  bool stable(uint8_t i, bool pressed);
  void pause(uint8_t i, Phase next);
  void fail(Error e);
};
}  // namespace scara
