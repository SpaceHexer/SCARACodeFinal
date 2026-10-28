#include "MotorController.h"
#include <Arduino.h>
#include <math.h>
#ifdef __AVR__
#include <avr/interrupt.h>
#endif
namespace scara {
namespace {
class Critical {
#ifdef __AVR__
  uint8_t saved_;
#endif
 public:
  Critical() {
#ifdef __AVR__
    saved_ = SREG;
    cli();
#endif
  }
  ~Critical() {
#ifdef __AVR__
    asm volatile("" ::: "memory");
    SREG = saved_;
    asm volatile("" ::: "memory");
#endif
  }
};
MotorController* active = nullptr;
}  // namespace
void MotorController::step(uint8_t i, bool high) {
#ifdef __AVR__
  if (high)
    *stepPort_[i] |= stepMask_[i];
  else
    *stepPort_[i] &= uint8_t(~stepMask_[i]);
#else
  digitalWrite(config::AXIS[i].stepPin, high ? HIGH : LOW);
#endif
}
void MotorController::direction(uint8_t i, int8_t sign) {
  bool high = sign * config::AXIS[i].direction > 0;
#ifdef __AVR__
  if (high)
    *dirPort_[i] |= dirMask_[i];
  else
    *dirPort_[i] &= uint8_t(~dirMask_[i]);
#else
  digitalWrite(config::AXIS[i].dirPin, high ? HIGH : LOW);
#endif
}
bool MotorController::limit(uint8_t i) const {
#ifdef __AVR__
  return (*limitPort_[i] & limitMask_[i]) == 0;
#else
  return digitalRead(config::AXIS[i].limitPin) == LOW;
#endif
}
void MotorController::enable(uint8_t mask) {
  enabledMask_ = mask ? uint8_t(enabledMask_ | mask) : 0;
  for (uint8_t i = 0; i < 4; ++i)
    digitalWrite(config::AXIS[i].enablePin, enabledMask_ & axisBit(i) ? LOW : HIGH);
}
void MotorController::begin() {
  Critical lock;
  for (uint8_t i = 0; i < 4; ++i) {
    const auto& a = config::AXIS[i];
    digitalWrite(a.enablePin, HIGH);
    pinMode(a.enablePin, OUTPUT);
    digitalWrite(a.stepPin, LOW);
    pinMode(a.stepPin, OUTPUT);
    digitalWrite(a.dirPin, LOW);
    pinMode(a.dirPin, OUTPUT);
    pinMode(a.limitPin, INPUT_PULLUP);
#ifdef __AVR__
    stepPort_[i] = portOutputRegister(digitalPinToPort(a.stepPin));
    stepMask_[i] = digitalPinToBitMask(a.stepPin);
    dirPort_[i] = portOutputRegister(digitalPinToPort(a.dirPin));
    dirMask_[i] = digitalPinToBitMask(a.dirPin);
    limitPort_[i] = portInputRegister(digitalPinToPort(a.limitPin));
    limitMask_[i] = digitalPinToBitMask(a.limitPin);
#endif
    framePulseLimit_[i] = config::maxFramePulses(i);
    homeAccel_[i] = uint16_t(fminf(10000, fmaxf(1, a.maxAcceleration * a.stepsPerUnit)));
  }
  if (stops_.pressedMask())
    halt(Error::Stopped);
  active = this;
#ifdef __AVR__
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = uint16_t(F_CPU / 8 / 1000000UL * config::TICK_US - 1);
  TIFR3 = _BV(OCF3A);
  TCCR3B = _BV(WGM32) | _BV(CS31);
  TIMSK3 = _BV(OCIE3A);
#endif
}
Error MotorController::ready() const {
  if (stops_.pressedMask())
    return Error::ButtonHeld;
  return error_;
}
Error MotorController::resetStop() {
  Critical lock;
  if (!stops_.canReset())
    return Error::ButtonHeld;
  if (running_ || homeMask_ || count_)
    return Error::Busy;
  error_ = Error::None;
  return Error::None;
}
MotorStatus MotorController::status() const {
  Critical lock;
  return {error_, running_, homeMask_};
}
Joints MotorController::position() const {
  int32_t n[4];
  {
    Critical lock;
    for (uint8_t i = 0; i < 4; ++i)
      n[i] = counts_[i];
  }
  Joints q;
  for (uint8_t i = 0; i < 4; ++i)
    q.q[i] = config::AXIS[i].qRef + n[i] / config::AXIS[i].stepsPerUnit;
  return q;
}
int32_t MotorController::steps(uint8_t i) const {
  if (i >= 4)
    return 0;
  Critical lock;
  return counts_[i];
}
bool MotorController::convert(const Joints& q, MotorFrame& f, uint8_t mask) const {
  for (uint8_t i = 0; i < 4; ++i) {
    const auto a = config::limits(i);
    if (!(mask & axisBit(i))) {
      f.target[i] = steps(i);
      continue;
    }
    if (!isfinite(q.q[i]) || q.q[i] < a.minimum - 1e-6f || q.q[i] > a.maximum + 1e-6f)
      return false;
    Real n = (q.q[i] - a.qRef) * a.stepsPerUnit;
    if (!isfinite(n) || fabsf(n) > 2147483000.0f)
      return false;
    f.target[i] = int32_t(lroundf(n));
  }
  return true;
}
bool MotorController::queueHasSpace() const {
  Critical lock;
  return count_ < config::FRAME_QUEUE_SIZE;
}
bool MotorController::enqueue(const MotorFrame& f) {
  Critical lock;
  if (error_ != Error::None || homeMask_ || count_ == config::FRAME_QUEUE_SIZE)
    return false;
  frames_[write_] = f;
  write_ = (write_ + 1) % config::FRAME_QUEUE_SIZE;
  ++count_;
  return true;
}
bool MotorController::startFrames(uint8_t mask) {
  Critical lock;
  if (ready() != Error::None || running_ || homeMask_ || !count_ || highMask_)
    return false;
  motionMask_ = mask;
  enable(mask);
  load_ = true;
  finishing_ = false;
  running_ = true;
  return true;
}
void MotorController::halt(Error reason) {
  error_ = reason;
  running_ = false;
  homeMask_ = 0;
  count_ = read_ = write_ = 0;
  for (uint8_t i = 0; i < 4; ++i)
    step(i, false);
  highMask_ = 0;
  enable(0);
}
void MotorController::emergencyStop(Error e) {
  Critical lock;
  halt(e);
}
bool MotorController::homeRun(uint8_t i, int8_t s, uint16_t rate, int32_t distance) {
  if (i >= 4 || (s != 1 && s != -1) || !rate || rate > 4500)
    return false;
  if (rate > config::AXIS[i].maxSpeed * config::AXIS[i].stepsPerUnit || distance < 0 ||
      distance > config::HOME_MAX_STEPS)
    return false;
  Critical lock;
  if (ready() != Error::None || running_ || count_ || (highMask_ & axisBit(i)))
    return false;
  sign_[i] = s;
  homeRate_[i] = rate;
  homeNow_[i] = 0;
  homePhase_[i] = homeRamp_[i] = 0;
  direction(i, s);
  if (distance) {
    homeFiniteMask_ |= axisBit(i);
    homeTarget_[i] = counts_[i] + s * distance;
  } else
    homeFiniteMask_ &= uint8_t(~axisBit(i));
  homeMask_ |= axisBit(i);
  enable(homeMask_);
  return true;
}
void MotorController::homeStop(uint8_t i) {
  Critical lock;
  homeMask_ &= uint8_t(~axisBit(i));
}
bool MotorController::homeQuiet(uint8_t i) const {
  Critical lock;
  return !(homeMask_ & axisBit(i)) && !(highMask_ & axisBit(i));
}
void MotorController::homeZero(uint8_t i) {
  Critical lock;
  if (homeQuiet(i))
    counts_[i] = 0;
}
bool MotorController::loadFrame() {
  if (!count_) {
    halt(Error::Underrun);
    return false;
  }
  current_ = frames_[read_];
  read_ = (read_ + 1) % config::FRAME_QUEUE_SIZE;
  --count_;
  for (uint8_t i = 0; i < 4; ++i) {
    int64_t delta = int64_t(current_.target[i]) - counts_[i];
    if (delta > framePulseLimit_[i] || delta < -int32_t(framePulseLimit_[i]) ||
        (!(motionMask_ & axisBit(i)) && delta)) {
      halt(Error::StepRate);
      return false;
    }
    distance_[i] = uint16_t(delta < 0 ? -delta : delta);
    sign_[i] = delta < 0 ? -1 : 1;
    acc_[i] = 0;
    direction(i, sign_[i]);
  }
  frameTick_ = 0;
  load_ = false;
  return true;
}
void MotorController::tick() {
  // Tasterprüfung steht vor allen Motoraktionen, auch im Leerlauf/Homing.
  if (stops_.pressedMask()) {
    if (error_ != Error::Stopped)
      halt(Error::Stopped);
    return;
  }
  uint32_t now = uint32_t(micros());
  if (clockStarted_ && (running_ || homeMask_) &&
      uint32_t(now - lastTick_) > config::TICK_US * 3 / 2) {
    halt(Error::Timing);
    lastTick_ = now;
    return;
  }
  lastTick_ = now;
  clockStarted_ = true;
  for (uint8_t i = 0; i < 4; ++i)
    if (highMask_ & axisBit(i))
      step(i, false);
  highMask_ = 0;
  if (error_ != Error::None)
    return;
  if (homeMask_) {
    for (uint8_t i = 0; i < 4; ++i)
      if (homeMask_ & axisBit(i)) {
        uint32_t remaining =
            (homeFiniteMask_ & axisBit(i))
                ? uint32_t(homeTarget_[i] > counts_[i] ? homeTarget_[i] - counts_[i]
                                                       : counts_[i] - homeTarget_[i])
                : 0;
        bool braking = (homeFiniteMask_ & axisBit(i)) &&
                       uint32_t(homeNow_[i]) * homeNow_[i] >= 2UL * homeAccel_[i] * remaining;
        homeRamp_[i] += homeAccel_[i];
        if (homeRamp_[i] >= 1000000UL / config::TICK_US) {
          homeRamp_[i] -= 1000000UL / config::TICK_US;
          if (braking && homeNow_[i] > 1)
            --homeNow_[i];
          else if (!braking && homeNow_[i] < homeRate_[i])
            ++homeNow_[i];
        }
        homePhase_[i] += homeNow_[i];
        if (homePhase_[i] >= 1000000UL / config::TICK_US) {
          homePhase_[i] -= 1000000UL / config::TICK_US;
          counts_[i] += sign_[i];
          step(i, true);
          highMask_ |= axisBit(i);
          if ((homeFiniteMask_ & axisBit(i)) && counts_[i] == homeTarget_[i])
            homeMask_ &= uint8_t(~axisBit(i));
        }
      }
    return;
  }
  if (!running_)
    return;
  for (uint8_t i = 0; i < 4; ++i)
    if ((motionMask_ & axisBit(i)) && limit(i)) {
      halt(Error::Endstop);
      return;
    }
  if (finishing_) {
    running_ = false;
    count_ = read_ = write_ = 0;
    return;
  }
  if (load_ && !loadFrame())
    return;
  // DDA: Jede Achse addiert ihre Impulszahl. Bei Erreichen der Frame-Taktzahl
  // entsteht ein Impuls. So passen alle Achsen in dieselbe Segmentzeit.
  for (uint8_t i = 0; i < 4; ++i) {
    acc_[i] += distance_[i];
    if (acc_[i] >= config::TICKS_PER_FRAME) {
      acc_[i] -= config::TICKS_PER_FRAME;
      counts_[i] += sign_[i];
      step(i, true);
      highMask_ |= axisBit(i);
    }
  }
  if (++frameTick_ == config::TICKS_PER_FRAME) {
    load_ = true;
    if (current_.last)
      finishing_ = true;
  }
}
#ifdef __AVR__
ISR(TIMER3_COMPA_vect) {
  if (active)
    active->tick();
}
#endif
}  // namespace scara
