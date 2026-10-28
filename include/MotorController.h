#pragma once
#include "StopInputs.h"

namespace scara {
  struct MotorFrame {
    int32_t target[4] = {};
    bool last = false;
  };

  struct MotorStatus {
    Error error;
    bool running;
    uint8_t homeMask;
  };

  class MotorController {
  public:
    explicit MotorController(StopInputs& stops) : stops_(stops) {}

    void begin();
    void tick();  // Timer3, keine IK/Serial-Ausgabe.
    Error ready() const;
    Error resetStop();
    void emergencyStop(Error reason = Error::Stopped);
    MotorStatus status() const;  // Atomarer Abschlussgrund statt !running allein.
    Joints position() const;
    int32_t steps(uint8_t axis) const;
    bool convert(const Joints& q, MotorFrame& frame, uint8_t mask = 15) const;
    bool enqueue(const MotorFrame& frame);
    bool queueHasSpace() const;
    bool startFrames(uint8_t mask);
    Error setReference(const Joints& knownPosition, uint8_t mask);  // Keine Motorbewegung.
    // distance=0: Endschalter-Suche; sonst endlicher Weg mit Anfahr-/Bremsrampe.
    bool homeRun(uint8_t axis, int8_t sign, uint16_t rate, int32_t distance = 0);
    void homeStop(uint8_t axis);
    bool homeQuiet(uint8_t axis) const;
    void homeZero(uint8_t axis);

  private:
    StopInputs& stops_;
    volatile Error error_ = Error::None;
    volatile bool running_ = false;
    volatile uint8_t homeMask_ = 0;
    volatile int32_t counts_[4] = {};

    // Umrechnung/Pruefung erfolgt beim Einreihen in loop(), nicht in der ISR.
    struct PreparedFrame {
      uint16_t distance[4] = {};
      uint8_t forwardMask = 0;
      bool last = false, valid = true;
    } frames_[config::FRAME_QUEUE_SIZE], current_;

    int32_t queuedTarget_[4] = {};  // Endpunkt des zuletzt eingereihten Frames.
    volatile uint8_t read_ = 0, write_ = 0, count_ = 0;
    uint8_t motionMask_ = 0, highMask_ = 0, enabledMask_ = 0, homeFiniteMask_ = 0;
    int32_t homeTarget_[4] = {};
    uint16_t framePulseLimit_[4] = {};
    uint16_t frameTick_ = 0, acc_[4] = {}, distance_[4] = {};
    int8_t sign_[4] = {1, 1, 1, 1};
    uint16_t homeRate_[4] = {}, homeNow_[4] = {}, homeAccel_[4] = {};
    uint32_t homePhase_[4] = {}, homeRamp_[4] = {};
    bool load_ = true, finishing_ = false, clockStarted_ = false;
    uint32_t lastTick_ = 0;
#ifdef __AVR__
    volatile uint8_t* stepPort_[4];
    volatile uint8_t* dirPort_[4];
    volatile uint8_t* limitPort_[4];
    uint8_t stepMask_[4], dirMask_[4], limitMask_[4];
#endif
    void step(uint8_t i, bool high);
    void direction(uint8_t i, int8_t sign);
    bool limit(uint8_t i) const;
    void enable(uint8_t mask);
    void halt(Error reason);
    bool loadFrame();
  };
}  // namespace scara
