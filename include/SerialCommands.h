#pragma once
#include "Console.h"
#include "Homing.h"
#include "Kinematics.h"
#include "Movement.h"
#include "PositionStore.h"

namespace scara {
  class SerialCommands {
  public:
    SerialCommands(MotorController& m, RobotState& r, Movement& move, Homing& h, PositionStore& store, StopInputs& stops) : motors_(m), robot_(r), movement_(move), homing_(h), store_(store), stops_(stops) {}

    void begin();
    void update();
    void reportEvents();

  private:
    MotorController& motors_;
    RobotState& robot_;
    Movement& movement_;
    Homing& homing_;
    PositionStore& store_;
    StopInputs& stops_;
    Kinematics kinematics_;
    Console out_;
    char buffer_[256];
    uint8_t length_ = 0;
    uint8_t listCursor_ = PositionStore::CAPACITY;
    bool discarding_ = false;
    static constexpr uint8_t MAX_CUE_ENTRIES = 16;
    struct CueEntry {
      uint8_t slot;
      uint32_t pauseMs;
    } cue_[MAX_CUE_ENTRIES];
    uint8_t cueCount_ = 0, cueIndex_ = 0;
    bool cueWaiting_ = false;
    uint32_t cuePauseStart_ = 0;
    Movement::State lastMove_ = Movement::Idle;
    Homing::State lastHome_ = Homing::Idle;
    Error lastFault_ = Error::None;
    void execute(char* line);
    void reply(Error e);
    void status();
    void pose(const Pose& p);
    Error ready(uint8_t mask) const;
    Error movePose(const Pose& p, Real duration);
    Error startCue(char* const* args, uint8_t n);
    Error moveCueEntry();
    void updateCue(Error motorError);
  };
}  // namespace scara
