#include "Kinematics.h"
#include <math.h>

namespace scara {
  Pose Kinematics::forward(const Joints& q) const {
    Pose p;
    p.z = q.q[3];
    p.psi = q.q[0] + q.q[1] + q.q[2];
    p.x = config::L1 * cosf(q.q[0]) + config::L2 * cosf(q.q[0] + q.q[1]) + config::TOOL_X * cosf(p.psi) - config::TOOL_Y * sinf(p.psi);
    p.y = config::L1 * sinf(q.q[0]) + config::L2 * sinf(q.q[0] + q.q[1]) + config::TOOL_X * sinf(p.psi) + config::TOOL_Y * cosf(p.psi);
    return p;
  }

  Error Kinematics::inverse(const Pose& p, const Joints& seed, Joints& out) const {
    Real values[] = {p.x, p.y, p.z, p.psi};
    for (Real v : values)
      if (!isfinite(v))
        return Error::BadArgs;
    for (Real v : seed.q)
      if (!isfinite(v))
        return Error::BadArgs;
    Real x = p.x - config::TOOL_X * cosf(p.psi) + config::TOOL_Y * sinf(p.psi);
    Real y = p.y - config::TOOL_X * sinf(p.psi) - config::TOOL_Y * cosf(p.psi);
    Real r = hypotf(x, y), inner = fabsf(config::L1 - config::L2), outer = config::L1 + config::L2;
    // Nur kleine, eindeutig numerische Randabweichungen tolerieren.
    if (r < inner - 2e-4f || r > outer + 2e-4f)
      return Error::Unreachable;
    if (r < 1e-5f && inner < 1e-5f)
      return Error::Degenerate;
    Real c = (x * x + y * y - config::L1 * config::L1 - config::L2 * config::L2) / (2 * config::L1 * config::L2);
    c = fmaxf(-1, fminf(1, c));
    Real best = INFINITY;
    bool found = false;
    // Beide Ellbogenlösungen prüfen. q1 zur nächsten Darstellung entrollen.
    // Liegt diese außerhalb der Grenzen, keine überraschende volle Umdrehung
    // als Ersatz erzwingen; große Gelenkfahrten ausdrücklich über MOVEJ wählen.
    for (int8_t branch = -1; branch <= 1; branch += 2) {
      Real q2 = branch * acosf(c), q1 = atan2f(y, x) - atan2f(config::L2 * sinf(q2), config::L1 + config::L2 * cosf(q2));
      Real turns = roundf((seed.q[0] - q1) / (2 * config::PI_VALUE));
      Joints q;
      q.q[0] = q1 + turns * 2 * config::PI_VALUE;
      q.q[1] = q2;
      q.q[2] = p.psi - q.q[0] - q.q[1];
      q.q[3] = p.z;
      bool valid = true;
      Real cost = 0;
      for (uint8_t i = 0; i < 4; ++i) {
        const auto a = config::limits(i);
        if (q.q[i] < a.minimum - 1e-6f || q.q[i] > a.maximum + 1e-6f)
          valid = false;
        Real delta = q.q[i] - seed.q[i];
        if (i < 3)
          cost += delta * delta;  // Z ist für beide Lösungen gleich; keine mm/rad-Mischung.
      }
      if (valid && cost < best) {
        out = q;
        best = cost;
        found = true;
      }
    }
    // Am gestreckten Rand ist die Ziel-IK zulässig: keine inverse Geschwindigkeits-
    // kinematik, keine laufende kartesische IK und kein Jacobian-Invertieren nötig.
    return found ? Error::None : Error::JointLimit;
  }
}  // namespace scara
