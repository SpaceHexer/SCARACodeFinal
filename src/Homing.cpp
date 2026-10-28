#include "Homing.h"
#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

namespace scara {
Error Homing::start(uint8_t mask, bool sequential) {
  if (!mask || (mask & 0xf0))
    return Error::BadArgs;
  if (busy() || motors_.status().running || motors_.status().homeMask)
    return Error::Busy;
  if (motors_.ready() != Error::None)
    return motors_.ready();
  for (uint8_t i = 0; i < 4; ++i)
    if (mask & axisBit(i)) {
      if (!config::HOME_CONFIRMED[i])
        return Error::NotConfigured;
      const auto& a = config::AXIS[i];
      if (!isfinite(a.stepsPerUnit) || a.stepsPerUnit <= 0 || !isfinite(a.qRef) ||
          !isfinite(a.maxSpeed) || a.maxSpeed <= 0 || !isfinite(a.maxAcceleration) ||
          a.maxAcceleration <= 0 || !a.clearanceSteps)
        return Error::BadArgs;
      if (!a.homeOffsetSteps || !a.seekSpeed || !a.latchSpeed ||
          (a.homeOffsetSteps > 0 ? 1 : -1) == a.homeDirection)
        return Error::BadArgs;
    }
  if (!sequential && (mask & (mask - 1))) {
    bool allowed = false;
    for (uint8_t group : config::PARALLEL_GROUPS)
      if ((mask & group) == mask)
        allowed = true;
    if (!config::PARALLEL_CONFIRMED || !allowed)
      return Error::NotConfigured;
  }
  robot_.invalidate(mask);
  pending_ = mask;
  active_ = 0;
  sequential_ = sequential;
  error_ = Error::None;
  state_ = Running;
  Error e = launchGroup();
  if (e != Error::None)
    fail(e);
  return e;
}
Error Homing::launchGroup() {
  uint8_t mask = pending_;
  if (sequential_) {
    // Achsnummern: J1=0, J2=1, J3=2, Z=3.
    const uint8_t order[] = {3, 0, 1, 2};

    mask = 0;
    for (uint8_t axis : order) {
      if (pending_ & axisBit(axis)) {  // Diese Achse wartet noch auf Homing.
        mask = axisBit(axis);        // Nur diese Achse als Nächstes starten.
        break;
      }
    }
  }
  active_ = mask;
  pending_ &= uint8_t(~mask);
  for (uint8_t i = 0; i < 4; ++i)
    if (mask & axisBit(i)) {
      axes_[i] = AxisHome{};
      auto& h = axes_[i];
      const auto& a = config::AXIS[i];
      h.started = millis();
      h.lastCount = motors_.steps(i);
      bool pressed = digitalRead(a.limitPin) == LOW;
      h.phase = pressed ? Preclear : FastSeek;
      if (!motors_.homeRun(i, pressed ? -a.homeDirection : a.homeDirection,
                           pressed ? a.latchSpeed : a.seekSpeed))
        return Error::Timing;
    }
  return Error::None;
}
bool Homing::stable(uint8_t i, bool pressed) {
  auto& h = axes_[i];
  bool actual = digitalRead(config::AXIS[i].limitPin) == LOW;
  if (actual != pressed) {
    h.observed = false;
    return false;
  }
  if (!h.observed) {
    h.observed = true;
    h.stableSince = millis();
  }
  return uint32_t(millis() - h.stableSince) >= config::SWITCH_DEBOUNCE_MS;
}
void Homing::pause(uint8_t i, Phase next) {
  motors_.homeStop(i);
  auto& h = axes_[i];
  h.phase = Pause;
  h.next = next;
  h.pauseSince = millis();
  h.observed = false;
}
void Homing::fail(Error e) {
  motors_.emergencyStop(e);
  robot_.invalidate();
  active_ = pending_ = 0;
  state_ = Failed;
  error_ = e;
}
void Homing::cancel() {
  active_ = pending_ = 0;
  state_ = Idle;
  error_ = Error::None;
}
void Homing::update() {
  if (!busy())
    return;
  Error e = motors_.status().error;
  if (e != Error::None) {
    fail(e);
    return;
  }
  for (uint8_t i = 0; i < 4; ++i)
    if (active_ & axisBit(i)) {
      auto& h = axes_[i];
      const auto& a = config::AXIS[i];
      int32_t count = motors_.steps(i);
      h.total += labs(count - h.lastCount);
      h.lastCount = count;
      if (uint32_t(millis() - h.started) > config::HOME_TIMEOUT_MS ||
          h.total > config::HOME_MAX_STEPS) {
        fail(Error::HomeTimeout);
        return;
      }
      switch (h.phase) {
        case Preclear:
          if (stable(i, false))
            pause(i, ClearFast);
          break;
        case FastSeek:
          if (stable(i, true))
            pause(i, Release);
          break;
        case Release:
          if (stable(i, false))
            pause(i, ClearSlow);
          break;
        case ClearFast:
          if (motors_.homeQuiet(i))
            pause(i, FastSeek);
          break;
        case ClearSlow:
          if (motors_.homeQuiet(i))
            pause(i, SlowSeek);
          break;
        case SlowSeek:
          if (stable(i, true))
            pause(i, Offset);
          break;
        case Pause:
          if (uint32_t(millis() - h.pauseSince) < 1 || !motors_.homeQuiet(i))
            break;
          h.phase = h.next;
          h.observed = false;
          if (h.phase == Offset) {
            motors_.homeZero(i);
            h.lastCount = 0;
            if (!motors_.homeRun(i, a.homeOffsetSteps < 0 ? -1 : 1, a.seekSpeed,
                                 labs(a.homeOffsetSteps))) {
              fail(Error::Timing);
              return;
            }
          } else if (h.phase == ClearFast || h.phase == ClearSlow) {
            if (!motors_.homeRun(i, -a.homeDirection, a.latchSpeed, a.clearanceSteps)) {
              fail(Error::Timing);
              return;
            }
          } else {
            int8_t direction = h.phase == Release ? -a.homeDirection : a.homeDirection;
            if (!motors_.homeRun(i, direction, h.phase == FastSeek ? a.seekSpeed : a.latchSpeed)) {
              fail(Error::Timing);
              return;
            }
          }
          break;
        case Offset:
          if (motors_.homeQuiet(i)) {
            h.phase = Finish;
            h.observed = false;
          }
          break;
        case Finish:
          if (digitalRead(a.limitPin) == LOW) {
            fail(Error::Endstop);
            return;
          }
          if (stable(i, false)) {
            motors_.homeZero(i);
            robot_.markReferenced(axisBit(i));
            active_ &= uint8_t(~axisBit(i));
            h.phase = Off;
          }
          break;
        default:
          break;
      }
    }
  if (!active_) {
    if (pending_) {
      Error error = launchGroup();
      if (error != Error::None)
        fail(error);
    } else
      state_ = Done;
  }
}
}  // namespace scara
