#include "Movement.h"
#include <math.h>

namespace scara {
  namespace {
    bool close(Real a, Real b) {
      return fabsf(a - b) <= 2e-5f;
    }

    bool rest(const Boundary& b) {
      for (uint8_t i = 0; i < 4; ++i)
        if (b.velocity.q[i] != 0 || b.acceleration.q[i] != 0)
          return false;
      return true;
    }

    Real effectiveSpeed(uint8_t i) {
      const auto a = config::limits(i);
      // Reserve für Rundung zweier Endpunkte; gleiche Pulsgrenze wie der Backend.
      Real pulses = Real(config::maxFramePulses(i)) - 1;
      return fminf(a.maxSpeed, pulses / (config::FRAME_SECONDS * a.stepsPerUnit));
    }
  }  // namespace

  Error Movement::begin() {
    if (busy() || motors_.status().running || motors_.status().homeMask)
      return Error::Busy;
    if (motors_.ready() != Error::None)
      return motors_.ready();
    count_ = plan_ = generate_ = attempt_ = motionMask_ = 0;
    generated_ = generateFrame_ = 0;
    started_ = false;
    totalDuration_ = 0;
    error_ = Error::None;
    state_ = Editing;
    return Error::None;
  }

  Error Movement::append(const MoveRequest& r) {
    if (state_ != Editing || count_ >= config::MAX_SEGMENTS)
      return Error::Busy;
    if (!r.mask || (r.mask & 0xf0) || !isfinite(r.T_wunsch) || r.T_wunsch <= 0 || r.T_wunsch > config::MAX_DURATION)
      return Error::BadArgs;
    if (!robot_.referenced(r.mask))
      return Error::NotHomed;
    Segment& s = segments_[count_];
    s = Segment{};
    if (count_)
      s.start = segments_[count_ - 1].goal;
    else
      s.start.position = motors_.position();
    s.goal = s.start;
    s.requested = r.T_wunsch;
    s.mask = r.mask;
    s.exact = r.exact;
    for (uint8_t i = 0; i < 4; ++i) {
      if (r.explicitStart && (!close(r.startVelocity.q[i], s.start.velocity.q[i]) || !close(r.startAcceleration.q[i], s.start.acceleration.q[i])))
        return Error::Continuity;
      if (!(r.mask & axisBit(i))) {
        // Eine ausgelassene Achse muss bereits im Stillstand sein.
        if (s.start.velocity.q[i] != 0 || s.start.acceleration.q[i] != 0)
          return Error::Continuity;
        continue;
      }
      const auto a = config::limits(i);
      Real v[] = {r.target.q[i], r.endVelocity.q[i], r.endAcceleration.q[i]};
      for (Real value : v)
        if (!isfinite(value))
          return Error::BadArgs;
      if (r.target.q[i] < a.minimum || r.target.q[i] > a.maximum)
        return Error::JointLimit;
      if (fabsf(r.endVelocity.q[i]) > effectiveSpeed(i) || fabsf(r.endAcceleration.q[i]) > a.maxAcceleration)
        return Error::ProfileLimit;
      s.goal.position.q[i] = r.target.q[i];
      s.goal.velocity.q[i] = r.endVelocity.q[i];
      s.goal.acceleration.q[i] = r.endAcceleration.q[i];
    }
    motionMask_ |= r.mask;
    ++count_;
    return Error::None;
  }

  Error Movement::run() {
    if (state_ != Editing || !count_)
      return Error::BadArgs;
    // Eine endende Fahrt kann weder mit v!=0 noch mit a!=0 in Halten übergehen.
    if (!rest(segments_[count_ - 1].goal))
      return Error::NeedSuccessor;
    if (motors_.ready() != Error::None)
      return motors_.ready();
    plan_ = attempt_ = 0;
    candidate_ = segments_[0].requested;
    state_ = Planning;
    return Error::None;
  }

  Error Movement::request(const MoveRequest& r) {
    Error e = begin();
    if (e != Error::None)
      return e;
    e = append(r);
    if (e == Error::None)
      e = run();
    if (e != Error::None)
      abort();
    return e;
  }

  void Movement::abort() {
    state_ = Idle;
    error_ = Error::None;
    count_ = 0;
  }

  void Movement::fail(Error e) {
    state_ = Failed;
    error_ = e;
  }

  Error Movement::prepare() {
    Segment& s = segments_[plan_];
    if (!attempt_ && rest(s.start) && rest(s.goal)) {
      // Exakte Maxima der Ruhe-zu-Ruhe-Form erlauben eine direkte Dauerabschätzung.
      for (uint8_t i = 0; i < 4; ++i)
        if (s.mask & axisBit(i)) {
          const auto a = config::limits(i);
          Real d = fabsf(s.goal.position.q[i] - s.start.position.q[i]);
          Real speed = effectiveSpeed(i);
          if (speed <= 0)
            return Error::ProfileLimit;
          Real need = fmaxf(1.875f * d / speed, sqrtf(5.773503f * d / a.maxAcceleration));
          if (a.maxJerk > 0)
            need = fmaxf(need, powf(60 * d / a.maxJerk, 1.0f / 3));
          candidate_ = fmaxf(candidate_, need * 1.0001f);
        }
    }
    Real T_probe = ceilf(candidate_ / config::FRAME_SECONDS - 1e-5f) * config::FRAME_SECONDS;
    if (T_probe < config::FRAME_SECONDS)
      T_probe = config::FRAME_SECONDS;
    if (T_probe > config::MAX_DURATION || (s.exact && fabsf(T_probe - s.requested) > 1e-5f))
      return Error::Duration;
    Error result = Error::None;
    for (uint8_t i = 0; i < 4; ++i) {
      if (!s.profile[i].configure(T_probe, s.start.position.q[i], s.start.velocity.q[i], s.start.acceleration.q[i], s.goal.position.q[i], s.goal.velocity.q[i], s.goal.acceleration.q[i]))
        return Error::BadArgs;
      if (s.mask & axisBit(i)) {
        AxisConfig limits = config::limits(i);
        limits.maxSpeed = effectiveSpeed(i);
        Error e = s.profile[i].check(limits);
        if (e != Error::None)
          result = e;
      }
    }
    if (result != Error::None) {
      if (s.exact || ++attempt_ >= 60)
        return result;
      // Bei festen nicht-null Randwerten immer NEU fitten; längere Dauer kann
      // Überschwingen verschärfen. Begrenzte Suche, kein Feasibility-Versprechen.
      candidate_ = T_probe * 1.08f;
      return Error::Busy;
    }
    s.duration = T_probe;
    s.frames = uint16_t(lroundf(T_probe / config::FRAME_SECONDS));
    totalDuration_ += T_probe;
    ++plan_;
    attempt_ = 0;
    if (plan_ < count_)
      candidate_ = segments_[plan_].requested;
    return Error::None;
  }

  Error Movement::frame() {
    Segment& s = segments_[generate_];
    Real t_lauf = (generateFrame_ + 1) * config::FRAME_SECONDS;
    Joints q;
    for (uint8_t i = 0; i < 4; ++i)
      q.q[i] = generateFrame_ + 1 == s.frames ? s.goal.position.q[i] : s.profile[i].evaluate(t_lauf).position;
    MotorFrame f;
    if (!motors_.convert(q, f, motionMask_))
      return Error::JointLimit;
    f.last = generate_ + 1 == count_ && generateFrame_ + 1 == s.frames;
    if (!motors_.enqueue(f))
      return Error::Underrun;
    ++generated_;
    if (++generateFrame_ == s.frames) {
      generateFrame_ = 0;
      ++generate_;
    }
    return Error::None;
  }

  void Movement::update() {
    if (state_ != Planning && state_ != Running)
      return;
    MotorStatus status = motors_.status();
    if (status.error != Error::None) {
      fail(status.error);
      return;
    }
    if (state_ == Planning) {
      Error e = prepare();
      if (e != Error::None && e != Error::Busy) {
        fail(e);
        return;
      }
      if (plan_ == count_)
        state_ = Running;
      return;
    }
    if (generate_ < count_ && motors_.queueHasSpace()) {
      Error e = frame();
      if (e != Error::None) {
        motors_.emergencyStop(e);
        fail(e);
        return;
      }
    }
    if (!started_ && (generated_ >= config::FRAME_QUEUE_SIZE || generate_ == count_)) {
      if (!motors_.startFrames(motionMask_))
        return;
      started_ = true;
    }
    // Erfolg und Fehler aus EINEM atomaren Snapshot lesen: keine DONE/ISR-Race.
    status = motors_.status();
    if (status.error != Error::None) {
      fail(status.error);
      return;
    }
    if (started_ && !status.running)
      state_ = Done;
  }
}  // namespace scara
