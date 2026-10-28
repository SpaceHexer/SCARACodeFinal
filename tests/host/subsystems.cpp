#include <Arduino.h>
#include "SerialCommands.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
using namespace scara;
void near(Real a, Real b, Real e = 3e-4f) {
  if (!isfinite(a) || !isfinite(b) || fabsf(a - b) > e) {
    printf("near failed %.9g %.9g tol %.9g\n", double(a), double(b), double(e));
    assert(false);
  }
}
void resetHardware() {
  testMicros = 0;
  simulateSwitches = false;
  Serial.input.clear();
  Serial.output.clear();
  memset(pins, 0, sizeof(pins));
  memset(modeCalls, 0, sizeof(modeCalls));
  memset(readCalls, 0, sizeof(readCalls));
  memset(writeCalls, 0, sizeof(writeCalls));
  memset(edges, 0, sizeof(edges));
  memset(drivenLow, 0, sizeof(drivenLow));
  memset(drivenHigh, 0, sizeof(drivenHigh));
  memset(physicalSteps, 0, sizeof(physicalSteps));
  txCapacity = 64;
}
struct Rig {
  StopInputs stops;
  MotorController motors;
  RobotState robot;
  Movement movement;
  Homing homing;
  PositionStore store;
  Rig() : motors(stops), movement(motors, robot), homing(motors, robot) {
    resetHardware();
    stops.begin();
    motors.begin();
  }
  void tick() {
    testMicros += config::TICK_US;
    txCapacity = 64;
    motors.tick();
    stops.poll();
  }
  void finish() {
    for (uint32_t n = 0; n < 4000000 && movement.busy(); ++n) {
      movement.update();
      tick();
    }
    assert(!movement.busy());
  }
};
void profiles() {
  QuinticProfile p;
  assert(p.configure(3, 0, 0, 0, 1, 0, 0));
  Real expected[] = {0, 0, 0, 10, -15, 6};
  for (int i = 0; i < 6; ++i)
    near(p.coefficients()[i], expected[i]);
  near(p.evaluate(1).position, 17.0f / 81);
  assert(p.configure(3, 0, 0, 0, 90, 30, 0));
  Sample s = p.evaluate(1.5);
  near(s.position, 30.9375f);
  near(s.velocity, 43.125f);
  near(s.acceleration, 15);
  for (int n = 1; n <= 100; ++n) {
    Real T = .2f + n * .037f, q0 = .003f * n, q1 = 1 + .001f * n, v0 = .001f * n, v1 = .003f * n,
         a0 = -.002f * n, a1 = .001f * n;
    assert(p.configure(T, q0, v0, a0, q1, v1, a1));
    s = p.evaluate(0);
    near(s.position, q0);
    near(s.velocity, v0);
    near(s.acceleration, a0);
    s = p.evaluate(T);
    near(s.position, q1);
    near(s.velocity, v1);
    near(s.acceleration, a1);
  }
  AxisConfig limits = config::AXIS[0];
  limits.minimum = -10;
  limits.maximum = 10;
  limits.maxSpeed = 100;
  limits.maxAcceleration = 100;
  limits.maxJerk = 0;
  assert(p.configure(1, 0, 20, 0, 0, -20, 0));
  assert(p.check(limits) == Error::None);
  limits.maximum = .5;
  assert(p.check(limits) == Error::JointLimit);  // Innenüberschwingen trotz gültiger Endpunkte.
  limits.maximum = 10;
  limits.maxSpeed = 1;
  assert(p.check(limits) == Error::ProfileLimit);
  limits.maxSpeed = 100;
  limits.maxAcceleration = 1;
  assert(p.check(limits) == Error::ProfileLimit);
  limits.maxAcceleration = 100;
  limits.maxJerk = 1;
  assert(p.check(limits) == Error::ProfileLimit);
  assert(!p.configure(0, 0, 0, 0, 1, 0, 0));
  assert(!p.configure(NAN, 0, 0, 0, 1, 0, 0));
  // Unabhängige dichte Testkontrolle; keine Freigabe durch bloße Stichproben.
  uint32_t rng = 137;
  uint16_t accepted = 0;
  auto random = [&]() {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return Real(rng % 10001) / 10000;
  };
  limits.minimum = -2;
  limits.maximum = 2;
  limits.maxSpeed = 2;
  limits.maxAcceleration = 3;
  limits.maxJerk = 20;
  for (uint16_t n = 0; n < 200; ++n) {
    Real T = .5f + 4 * random();
    assert(p.configure(T, 2 * random() - 1, random() - .5f, random() - .5f, 2 * random() - 1,
                       random() - .5f, random() - .5f));
    if (p.check(limits) != Error::None)
      continue;
    ++accepted;
    for (uint16_t k = 0; k <= 4000; ++k) {
      Sample sample = p.evaluate(T * k / 4000);
      assert(sample.position >= -2.0001f && sample.position <= 2.0001f);
      assert(fabsf(sample.velocity) <= 2.0001f && fabsf(sample.acceleration) <= 3.0001f &&
             fabsf(sample.jerk) <= 20.0005f);
    }
  }
  assert(accepted > 10);
  printf("PASS dense cross-check: %u accepted float32 general profiles\n", accepted);
  puts(
      "PASS quintic: all six boundaries, interior extrema, speed/acceleration/jerk limits "
      "(float32)");
}
void kinematics() {
  Kinematics k;
  Joints seed, q;
  Pose p;
  p.x = config::L1 + config::L2;
  assert(k.inverse(p, seed, q) == Error::None);
  near(q.q[1], 0);  // Gestrecktes Ziel zulässig.
  p.x = 0;
  p.y = 0;
  assert(k.inverse(p, seed, q) == Error::Unreachable);
  for (int branch : {-1, 1})
    for (int n = 0; n < 100; ++n) {
      seed.q[0] = -.7f + .01f * n;
      seed.q[1] = branch > 0 ? (.3f + .015f * n) : (-.025f - .0005f * n);
      seed.q[2] = -seed.q[0] - seed.q[1];
      seed.q[3] = 20;
      p = k.forward(seed);
      assert(k.inverse(p, seed, q) == Error::None);
      for (int i = 0; i < 4; ++i)
        near(q.q[i], seed.q[i], 2e-4f);
    }
  p.x = 500;
  p.y = 0;
  assert(k.inverse(p, seed, q) == Error::Unreachable);
  p = k.forward(seed);
  p.z = 400;
  assert(k.inverse(p, seed, q) == Error::JointLimit);
  p.x = 210;
  p.y = 145;
  p.z = 20;
  p.psi = 0;
  Joints zero;
  assert(k.inverse(p, zero, q) == Error::None);
  assert(q.q[1] >= config::limits(1).minimum);
  puts("PASS endpoint IK/FK: branches, limits, straight-arm target, no Cartesian start planner");
}
void stopInputs() {
  for (int button = 0; button < 2; ++button) {
    Rig r;
    uint8_t pin = button ? config::STOP_BUTTON_2_PIN : config::STOP_BUTTON_1_PIN;
    bool enabled = button ? config::STOP_BUTTON_2_ENABLED : config::STOP_BUTTON_1_ENABLED;
    drivenLow[pin] = true;
    r.tick();
    assert((r.motors.status().error == Error::Stopped) == enabled);
    if (enabled) {
      assert(r.motors.resetStop() == Error::ButtonHeld);
      drivenLow[pin] = false;
      for (int n = 0; n < 250; ++n)
        r.tick();
      assert(r.motors.status().error == Error::Stopped);
      assert(r.motors.resetStop() == Error::None);
    } else {
      assert(modeCalls[pin] == 0 && readCalls[pin] == 0 && writeCalls[pin] == 0);
      r.motors.emergencyStop();  // Auch unmittelbar nach Boot kein Phantom-HELD.
      assert(r.motors.resetStop() == Error::None);
      assert(modeCalls[pin] == 0 && readCalls[pin] == 0 && writeCalls[pin] == 0);
    }
  }
  resetHardware();
  drivenLow[config::STOP_BUTTON_1_PIN] = true;
  drivenLow[config::STOP_BUTTON_2_PIN] = true;
  StopInputs stops;
  stops.begin();
  MotorController m(stops);
  m.begin();
  assert((m.status().error == Error::Stopped) ==
         (config::STOP_BUTTON_1_ENABLED || config::STOP_BUTTON_2_ENABLED));
  if (!config::STOP_BUTTON_1_ENABLED)
    assert(!modeCalls[config::STOP_BUTTON_1_PIN] && !readCalls[config::STOP_BUTTON_1_PIN]);
  if (!config::STOP_BUTTON_2_ENABLED)
    assert(!modeCalls[config::STOP_BUTTON_2_PIN] && !readCalls[config::STOP_BUTTON_2_PIN]);
  puts(
      "PASS STOP configuration: disabled pins untouched, boot hold, reset interlock, no auto "
      "resume");
}
void motor() {
  Rig r;
  Joints q;
  MotorFrame f;
  q.q[0] = config::PI_VALUE / 2;
  q.q[1] = config::PI_VALUE / 2;
  q.q[2] = config::PI_VALUE / 2;
  q.q[3] = 30;
  assert(r.motors.convert(q, f));
  assert(f.target[0] == 4000 && f.target[1] == 3200 && f.target[2] == 800 && f.target[3] == 3000);
  f.target[0] = 20;
  f.target[1] = -10;
  f.target[2] = 5;
  f.target[3] = 30;
  f.last = true;
  assert(r.motors.enqueue(f) && r.motors.startFrames(15));
  for (int n = 0; n <= config::TICKS_PER_FRAME; ++n)
    r.tick();
  int expected[] = {20, -10, 5, 30};
  assert(!r.motors.status().running && r.motors.status().error == Error::None);
  for (int i = 0; i < 4; ++i) {
    assert(r.motors.steps(i) == expected[i]);
    assert(edges[config::AXIS[i].stepPin] == uint32_t(abs(expected[i])));
  }
  f.last = false;
  assert(r.motors.enqueue(f) && r.motors.startFrames(15));
  for (int n = 0; n <= config::TICKS_PER_FRAME; ++n)
    r.tick();
  assert(r.motors.status().error == Error::Underrun);
  for (int i = 0; i < 4; ++i)
    assert(pins[config::AXIS[i].enablePin] == HIGH);
  assert(r.motors.resetStop() == Error::None);
  f.last = true;
  f.target[0] = r.motors.steps(0) + 1;
  assert(r.motors.enqueue(f) && r.motors.startFrames(15));
  r.tick();
  testMicros += 3 * config::TICK_US;
  r.motors.tick();
  assert(r.motors.status().error == Error::Timing);
  assert(r.motors.resetStop() == Error::None);
  f.target[0] = r.motors.steps(0) + config::maxFramePulses(0) + 1;
  assert(r.motors.enqueue(f) && r.motors.startFrames(15));
  r.tick();
  assert(r.motors.status().error == Error::StepRate);
  assert(r.motors.resetStop() == Error::None);
  // Extreme absolute Ziele duerfen weder ueberlaufen noch als kurze Differenz
  // in die ISR gelangen. Auch danach ist keine erfolgreiche Bewegung erlaubt.
  f.target[0] = INT32_MIN;
  assert(r.motors.enqueue(f) && r.motors.startFrames(15));
  r.tick();
  assert(r.motors.status().error == Error::StepRate);
  assert(r.motors.resetStop() == Error::None);
  f.target[0] = INT32_MAX;
  assert(r.motors.enqueue(f) && r.motors.startFrames(15));
  r.tick();
  assert(r.motors.status().error == Error::StepRate);
  puts("PASS direct pulse counts, common time, underrun/timing/rate terminal errors");
}
void movements() {
  Rig r;
  r.robot.markReferenced(15);
  MoveRequest q;
  q.target.q[0] = .2f;
  q.target.q[1] = 1;
  q.target.q[2] = -1;
  q.target.q[3] = 20;
  q.T_wunsch = .1f;
  assert(r.movement.request(q) == Error::None);
  r.finish();
  assert(r.movement.state() == Movement::Done);
  assert(r.movement.duration() > .1);
  Joints end = r.motors.position();
  for (int i = 0; i < 4; ++i)
    near(end.q[i], q.target.q[i], .51f / config::AXIS[i].stepsPerUnit);
  q.exact = true;
  q.T_wunsch = .02f;
  q.target.q[0] = 1;
  assert(r.movement.request(q) == Error::None);
  r.finish();
  assert(r.movement.state() == Movement::Failed);
  q = MoveRequest{};
  q.mask = 1;
  q.target = r.motors.position();
  q.target.q[0] = .3f;
  q.T_wunsch = 2;
  q.endVelocity.q[0] = .02f;
  assert(r.movement.begin() == Error::None);
  assert(r.movement.append(q) == Error::None);
  assert(r.movement.run() == Error::NeedSuccessor);
  MoveRequest next = q;
  next.target.q[0] = .4f;
  next.endVelocity.q[0] = 0;
  next.explicitStart = true;
  next.startVelocity.q[0] = .03f;
  assert(r.movement.append(next) == Error::Continuity);
  next.startVelocity.q[0] = .02f;
  assert(r.movement.append(next) == Error::None);
  assert(r.movement.run() == Error::None);
  r.finish();
  assert(r.movement.state() == Movement::Done);
  near(r.motors.position().q[0], .4f, .51f / config::K1);
  q.endVelocity.q[0] = 1e-7f;
  assert(r.movement.begin() == Error::None);
  assert(r.movement.append(q) == Error::None);
  assert(r.movement.run() == Error::NeedSuccessor);
  r.movement.abort();
  // Nicht-null Startbeschleunigung im Anschluss wird ebenfalls wirklich genutzt.
  q = MoveRequest{};
  q.mask = 1;
  q.target = r.motors.position();
  q.target.q[0] = .5f;
  q.T_wunsch = 2;
  q.endAcceleration.q[0] = .01f;
  assert(r.movement.begin() == Error::None);
  assert(r.movement.append(q) == Error::None);
  next = q;
  next.target.q[0] = .6f;
  next.endAcceleration.q[0] = 0;
  next.explicitStart = true;
  next.startAcceleration.q[0] = .01f;
  assert(r.movement.append(next) == Error::None);
  assert(r.movement.run() == Error::None);
  r.finish();
  assert(r.movement.state() == Movement::Done);
  puts(
      "PASS synchronized joints, EXACT, executable v/a boundary chains, successor and continuity "
      "checks");
}
void partialAndStore() {
  Rig r;
  MoveRequest q;
  q.mask = 1;
  q.target.q[0] = .1f;
  assert(r.movement.request(q) == Error::NotHomed);
  r.robot.markReferenced(1);
  assert(r.movement.request(q) == Error::None);
  r.finish();
  assert(r.movement.state() == Movement::Done);
  assert(!r.robot.referenced(15));
  Pose p;
  p.x = 210;
  p.y = 145;
  assert(r.store.load(0, p) == Error::EmptySlot);
  assert(r.store.save(0, p) == Error::None);
  Pose out;
  assert(r.store.load(0, out) == Error::None);
  near(out.x, 210);
  assert(r.store.erase(0) == Error::None);
  assert(r.store.load(0, out) == Error::EmptySlot);
  assert(r.store.save(16, p) == Error::BadArgs);
  PositionStore restarted;
  assert(restarted.load(0, out) == Error::EmptySlot);
  puts("PASS partial reference gating and RAM position lifecycle");
}
void home() {
  for (bool sequential : {false, true}) {
    Rig r;
    simulateSwitches = true;
    assert(r.homing.start(15, sequential) == Error::None);
    bool sawParallel = false;
    for (uint32_t n = 0; n < 4000000 && r.homing.busy(); ++n) {
      r.tick();
      r.homing.update();
      uint8_t mask = r.motors.status().homeMask;
      if (mask && (mask & (mask - 1)))
        sawParallel = true;
    }
    assert(r.homing.state() == Homing::Done && r.robot.referenced(15));
    assert(sawParallel == !sequential);
    for (int i = 0; i < 4; ++i) {
      assert(r.motors.steps(i) == 0);
      assert(digitalRead(config::AXIS[i].limitPin) == HIGH);
    }
  }
  Rig r;
  simulateSwitches = true;
  assert(r.homing.start(1, true) == Error::None);
  for (uint32_t n = 0; n < 4000000 && r.homing.busy(); ++n) {
    r.tick();
    r.homing.update();
  }
  assert(r.robot.referenced(1) && !r.robot.referenced(15));
  r.robot.invalidate();
  simulateSwitches = false;
  assert(r.homing.start(1, true) == Error::None);
  testMicros += uint64_t(config::HOME_TIMEOUT_MS + 1) * 1000;
  r.homing.update();
  assert(r.homing.error() == Error::HomeTimeout && !r.robot.referenced(1));
  puts("PASS single, sequential and parallel two-pass homing; timeout invalidates references");
}
int main() {
  profiles();
  kinematics();
  stopInputs();
  motor();
  movements();
  partialAndStore();
  home();
  puts("All subsystem tests passed");
}
