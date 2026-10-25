#include "QuinticProfile.h"
#include <math.h>
namespace scara {
namespace {
Real value(const Real* c, uint8_t degree, Real x) {
  Real v = c[degree];
  while (degree)
    v = v * x + c[--degree];
  return v;
}
void addRoot(Real* roots, uint8_t& count, Real r) {
  if (r < 0 || r > 1 || count >= 4)
    return;
  for (uint8_t i = 0; i < count; ++i)
    if (fabsf(roots[i] - r) < 2e-6f)
      return;
  uint8_t i = count++;
  while (i && roots[i - 1] > r) {
    roots[i] = roots[i - 1];
    --i;
  }
  roots[i] = r;
}
uint8_t rootsInUnit(const Real* c, uint8_t degree, Real* roots) {
  while (degree && c[degree] == 0)
    --degree;
  if (!degree)
    return 0;
  uint8_t count = 0;
  if (degree == 1) {
    addRoot(roots, count, -c[0] / c[1]);
    return count;
  }
  Real derivative[5] = {}, critical[4] = {};
  for (uint8_t i = 1; i <= degree; ++i)
    derivative[i - 1] = i * c[i];
  uint8_t n = rootsInUnit(derivative, degree - 1, critical);
  Real points[6] = {};
  uint8_t size = 1;
  for (uint8_t i = 0; i < n; ++i)
    if (critical[i] > 0 && critical[i] < 1)
      points[size++] = critical[i];
  points[size++] = 1;
  // Zwischen benachbarten Ableitungsnullstellen ist das Polynom monoton.
  for (uint8_t i = 0; i < size; ++i) {
    Real f = value(c, degree, points[i]);
    if (f == 0)
      addRoot(roots, count, points[i]);
    if (i == 0)
      continue;
    Real lo = points[i - 1], hi = points[i], fl = value(c, degree, lo), fh = f;
    if ((fl < 0 && fh > 0) || (fl > 0 && fh < 0)) {
      for (uint8_t k = 0; k < 32; ++k) {
        Real mid = (lo + hi) / 2;
        if (mid == lo || mid == hi)
          break;
        Real fm = value(c, degree, mid);
        if (fm == 0) {
          lo = hi = mid;
          break;
        }
        if ((fl < 0 && fm > 0) || (fl > 0 && fm < 0))
          hi = mid;
        else {
          lo = mid;
          fl = fm;
        }
      }
      addRoot(roots, count, (lo + hi) / 2);
    }
  }
  return count;
}
}  // namespace
bool QuinticProfile::configure(Real T_fahrt, Real q0, Real v0, Real a0, Real q1, Real v1, Real a1) {
  Real input[] = {T_fahrt, q0, v0, a0, q1, v1, a1};
  for (Real v : input)
    if (!isfinite(v))
      return false;
  if (T_fahrt <= 0)
    return false;
  Real b[6];
  b[0] = q0;
  b[1] = T_fahrt * v0;
  b[2] = T_fahrt * T_fahrt * a0 / 2;
  Real R = q1 - b[0] - b[1] - b[2], V = T_fahrt * v1 - b[1] - 2 * b[2],
       A = T_fahrt * T_fahrt * a1 - 2 * b[2];
  b[3] = 10 * R - 4 * V + A / 2;
  b[4] = -15 * R + 7 * V - A;
  b[5] = 6 * R - 3 * V + A / 2;
  for (Real v : b)
    if (!isfinite(v))
      return false;
  for (uint8_t i = 0; i < 6; ++i)
    b_[i] = b[i];
  duration_ = T_fahrt;
  endPosition_ = q1;
  endVelocity_ = v1;
  endAcceleration_ = a1;
  return true;
}
Sample QuinticProfile::evaluate(Real t_lauf) const {
  if (duration_ <= 0 || !isfinite(t_lauf))
    return {NAN, NAN, NAN, NAN};
  Real u = fmaxf(0, fminf(1, t_lauf / duration_));
  Sample s;
  s.position = value(b_, 5, u);
  s.velocity =
      ((((5 * b_[5] * u + 4 * b_[4]) * u + 3 * b_[3]) * u + 2 * b_[2]) * u + b_[1]) / duration_;
  s.acceleration =
      (((20 * b_[5] * u + 12 * b_[4]) * u + 6 * b_[3]) * u + 2 * b_[2]) / (duration_ * duration_);
  s.jerk = ((60 * b_[5] * u + 24 * b_[4]) * u + 6 * b_[3]) / (duration_ * duration_ * duration_);
  if (t_lauf >= duration_) {
    s.position = endPosition_;
    s.velocity = endVelocity_;
    s.acceleration = endAcceleration_;
  }
  return s;
}
Error QuinticProfile::check(const AxisConfig& a) const {
  if (duration_ <= 0 || !isfinite(a.minimum) || !isfinite(a.maximum) || a.minimum > a.maximum ||
      !isfinite(a.maxSpeed) || !isfinite(a.maxAcceleration) || !isfinite(a.maxJerk) ||
      a.maxSpeed <= 0 || a.maxAcceleration <= 0 || a.maxJerk < 0)
    return Error::BadArgs;
  Real c[6];
  for (uint8_t i = 0; i < 6; ++i)
    c[i] = b_[i];
  uint8_t degree = 5;
  for (uint8_t order = 0; order < 4; ++order) {
    Real d[5] = {}, roots[4] = {};
    for (uint8_t i = 1; i <= degree; ++i)
      d[i - 1] = i * c[i];
    uint8_t n = rootsInUnit(d, degree - 1, roots);
    for (uint8_t k = 0; k < n + 2; ++k) {
      Real u = k == 0 ? 0 : k == 1 ? 1 : roots[k - 2];
      Sample s = evaluate(u * duration_);
      Real v = order == 0   ? s.position
               : order == 1 ? s.velocity
               : order == 2 ? s.acceleration
                            : s.jerk;
      if (!isfinite(v))
        return Error::ProfileLimit;
      if (order == 0) {
        if (v < a.minimum - 1e-6f || v > a.maximum + 1e-6f)
          return Error::JointLimit;
      } else {
        Real limit = order == 1 ? a.maxSpeed : order == 2 ? a.maxAcceleration : a.maxJerk;
        if (limit > 0 && fabsf(v) > limit * (1 + 2e-5f))
          return Error::ProfileLimit;
      }
    }
    for (uint8_t i = 0; i < degree; ++i)
      c[i] = d[i];
    --degree;
  }
  return Error::None;
}
}  // namespace scara
