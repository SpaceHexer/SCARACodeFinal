#pragma once
#include "MotorController.h"
#include "RobotState.h"
#include "QuinticProfile.h"

namespace scara {
  struct MoveRequest {
    Joints target, startVelocity, startAcceleration, endVelocity, endAcceleration;
    Real T_wunsch = config::DEFAULT_DURATION;
    uint8_t mask = 15;
    bool explicitStart = false, exact = false;
  };

  class Movement {
  public:
    enum State : uint8_t { Idle, Editing, Planning, Running, Done, Failed };

    Movement(MotorController& m, RobotState& r) : motors_(m), robot_(r) {}

    Error begin();
    Error append(const MoveRequest& request);
    Error run();
    Error request(const MoveRequest& request);
    void abort();  // Nur nach Motor-STOP oder vor Ausführung verwenden.
    void update();

    bool busy() const {
      return state_ == Editing || state_ == Planning || state_ == Running;
    }

    State state() const {
      return state_;
    }

    Error error() const {
      return error_;
    }

    Real duration() const {
      return totalDuration_;
    }

    uint8_t segmentCount() const {
      return count_;
    }

  private:
    struct Segment {
      Boundary start, goal;
      QuinticProfile profile[4];
      Real requested = 0, duration = 0;
      uint16_t frames = 0;
      uint8_t mask = 0;
      bool exact = false;
    } segments_[config::MAX_SEGMENTS];

    MotorController& motors_;
    RobotState& robot_;
    State state_ = Idle;
    Error error_ = Error::None;
    uint8_t count_ = 0, plan_ = 0, generate_ = 0, attempt_ = 0, motionMask_ = 0;
    uint16_t generateFrame_ = 0;
    uint32_t generated_ = 0;
    Real candidate_ = 0, totalDuration_ = 0;
    bool started_ = false;
    Error prepare();
    Error frame();
    void fail(Error e);
  };
}  // namespace scara
