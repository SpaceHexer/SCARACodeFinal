#include "SerialCommands.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
namespace scara {
namespace {
void upper(char* p) {
  for (; *p; ++p)
    *p = char(toupper(static_cast<unsigned char>(*p)));
}
bool number(const char* p, Real& result) {
  if (!p || !*p)
    return false;
  char* end;
  double v = strtod(p, &end);
  result = Real(v);
  return end != p && !*end && isfinite(v) && isfinite(result);
}
int8_t axis(const char* p) {
  if (!strcmp(p, "J1"))
    return 0;
  if (!strcmp(p, "J2"))
    return 1;
  if (!strcmp(p, "J3"))
    return 2;
  if (!strcmp(p, "Z"))
    return 3;
  return -1;
}
bool slot(const char* p, uint8_t& id) {
  Real value;
  if (!number(p, value) || value < 0 || value >= PositionStore::CAPACITY || floorf(value) != value)
    return false;
  id = uint8_t(value);
  return true;
}
}  // namespace
void SerialCommands::begin() {
  Serial.begin(config::SERIAL_BAUD);
  out_.println(F("SCARA JOINT CONTROLLER; HELP"));
  out_.print(F("STOP1="));
  out_.print(config::STOP_BUTTON_1_ENABLED ? 1 : 0);
  out_.print(F(" STOP2="));
  out_.println(config::STOP_BUTTON_2_ENABLED ? 1 : 0);
  out_.println(F("Positionen: RAM, nach Neustart leer"));
}
void SerialCommands::reply(Error e) {
  if (e == Error::None)
    out_.println(F("OK"));
  else {
    out_.print(F("ERR "));
    out_.println(errorName(e));
  }
}
Error SerialCommands::ready(uint8_t mask) const {
  Error e = motors_.ready();
  if (e != Error::None)
    return e;
  if (movement_.busy() || homing_.busy())
    return Error::Busy;
  return robot_.referenced(mask) ? Error::None : Error::NotHomed;
}
void SerialCommands::pose(const Pose& p) {
  out_.print(F("X="));
  out_.print(p.x, 3);
  out_.print(F(" Y="));
  out_.print(p.y, 3);
  out_.print(F(" Z="));
  out_.print(p.z, 3);
  out_.print(F(" PSI_DEG="));
  out_.println(p.psi / config::RAD_PER_DEG, 3);
}
void SerialCommands::status() {
  out_.print(F("STATE REF_MASK="));
  out_.print(robot_.referenceMask());
  out_.print(F(" BUSY="));
  out_.print(movement_.busy() || homing_.busy() ? 1 : 0);
  out_.print(F(" FAULT="));
  out_.println(errorName(motors_.status().error));
  out_.print(F("STOP1="));
  out_.print(config::STOP_BUTTON_1_ENABLED ? 1 : 0);
  out_.print(F(" STOP2="));
  out_.print(config::STOP_BUTTON_2_ENABLED ? 1 : 0);
  out_.print(F(" PRESSED="));
  out_.println(stops_.pressedMask());
  out_.print(F("ESTIMATED J1/J2/J3_DEG,Z_MM: "));
  Joints q = motors_.position();
  for (uint8_t i = 0; i < 4; ++i) {
    if (robot_.referenced(axisBit(i)))
      out_.print(i < 3 ? q.q[i] / config::RAD_PER_DEG : q.q[i], 3);
    else
      out_.print('?');
    out_.print(' ');
  }
  out_.println();
  if (robot_.referenced(15))
    pose(kinematics_.forward(q));
}
Error SerialCommands::movePose(const Pose& p, Real T) {
  Error e = ready(15);
  if (e != Error::None)
    return e;
  if (!config::GEOMETRY_CONFIRMED)
    return Error::NotConfigured;
  MoveRequest r;
  r.T_wunsch = T;
  e = kinematics_.inverse(p, motors_.position(), r.target);
  if (e != Error::None)
    return e;
  lastMove_ = Movement::Idle;
  return movement_.request(r);
}
void SerialCommands::execute(char* line) {
  char* t[32];
  uint8_t n = 0;
  for (char* token = strtok(line, " \t"); token; token = strtok(nullptr, " \t")) {
    if (n == 32) {
      reply(Error::BadArgs);
      return;
    }
    upper(token);
    t[n++] = token;
  }
  if (!n)
    return;
  const char* cmd = t[0];
  if (!strcmp(cmd, "STOP") && n == 1) {
    motors_.emergencyStop();
    movement_.abort();
    homing_.cancel();
    robot_.invalidate();
    lastMove_ = Movement::Idle;
    lastHome_ = Homing::Idle;
    lastFault_ = Error::Stopped;
    out_.println(F("STOPPED; RESETSTOP, dann HOME oder SETREF erforderlich"));
    return;
  }
  if (!strcmp(cmd, "RESETSTOP") && n == 1) {
    if (movement_.busy() || homing_.busy()) {
      reply(Error::Busy);
      return;
    }
    Error e = motors_.resetStop();
    if (e == Error::None) {
      robot_.invalidate();
      movement_.abort();
      homing_.cancel();
      lastFault_ = Error::None;
    }
    reply(e);
    return;
  }
  if (!strcmp(cmd, "PING") && n == 1) {
    out_.println(F("PONG"));
    return;
  }
  if ((!strcmp(cmd, "STATUS") || !strcmp(cmd, "GETSTATE")) && n == 1) {
    status();
    return;
  }
  if (!strcmp(cmd, "HELP") && n == 1) {
    out_.println(F("HOME ALL [SEQUENTIAL] | HOME J1/J2/J3/Z"));
    out_.println(F("SETREF ALL j1 j2 j3 z | SETREF J1/J2/J3/Z wert (bekannte Iststellung)"));
    out_.println(F("SETJ J1 deg [T] | SETZ mm [T] | MOVEJ q1 q2 q3 z T"));
    out_.println(F("MOVEXY x y [Z mm] [PSI deg] [T s]"));
    out_.println(F("SAVE n | POSITION n [T] | LIST | DELETE n"));
    out_.println(F("BEGIN | SEG q1 q2 q3 z T [V0/A0/V1/A1 4 Werte] [EXACT] | RUN | CANCEL"));
    out_.println(F("STOP | RESETSTOP | STATUS | FK | IK | COEFF | PROFILE"));
    return;
  }
  if (!strcmp(cmd, "MOVEL") || !strcmp(cmd, "CHECKLINE")) {
    out_.println(F("ERR REMOVED: keine kartesische Streckenplanung; MOVEXY nutzen"));
    return;
  }
  if (!strcmp(cmd, "SETREF")) {
    // Manuelle Referenzierung: kein Verfahren zum Ziel, sondern Uebernahme
    // einer vom Benutzer bekannten aktuellen Stellung. Winkel in Grad, Z in mm.
    if (movement_.busy() || homing_.busy()) {
      reply(Error::Busy);
      return;
    }
    Joints q;
    uint8_t mask = 0;
    if (n == 6 && !strcmp(t[1], "ALL")) {
      mask = 15;
      for (uint8_t i = 0; i < 4; ++i) {
        if (!number(t[2 + i], q.q[i])) {
          reply(Error::BadArgs);
          return;
        }
        if (i < 3)
          q.q[i] *= config::RAD_PER_DEG;
      }
    } else if (n == 3 && axis(t[1]) >= 0) {
      const uint8_t i = uint8_t(axis(t[1]));
      mask = axisBit(i);
      if (!number(t[2], q.q[i])) {
        reply(Error::BadArgs);
        return;
      }
      if (i < 3)
        q.q[i] *= config::RAD_PER_DEG;
    } else {
      reply(Error::BadArgs);
      return;
    }
    const Error e = motors_.setReference(q, mask);
    if (e == Error::None) {
      robot_.markReferenced(mask);
      out_.println(F("OK SETREF: Stellung manuell uebernommen, nicht gemessen"));
    } else
      reply(e);
    return;
  }
  if (!strcmp(cmd, "HOME")) {
    if (movement_.busy()) {
      reply(Error::Busy);
      return;
    }
    uint8_t mask = 15;
    bool seq = false;
    if (n == 1) {
    } else if (n == 2 && !strcmp(t[1], "ALL")) {
    } else if (n == 3 && !strcmp(t[1], "ALL") && !strcmp(t[2], "SEQUENTIAL"))
      seq = true;
    else if (n == 2 && axis(t[1]) >= 0) {
      mask = axisBit(uint8_t(axis(t[1])));
      seq = true;
    } else {
      reply(Error::BadArgs);
      return;
    }
    const Error e = homing_.start(mask, seq);
    // Abgelehnte Auftraege erzeugen keinen neuen Abschluss. Andernfalls
    // koennte ein vorheriges Done faelschlich erneut als DONE HOME erscheinen.
    if (e == Error::None)
      lastHome_ = Homing::Idle;
    reply(e);
    return;
  }
  if (!strcmp(cmd, "BEGIN") && n == 1) {
    if (homing_.busy()) {
      reply(Error::Busy);
      return;
    }
    lastMove_ = Movement::Idle;
    reply(movement_.begin());
    return;
  }
  if (!strcmp(cmd, "RUN") && n == 1) {
    reply(movement_.run());
    return;
  }
  if (!strcmp(cmd, "CANCEL") && n == 1) {
    if (movement_.state() != Movement::Editing) {
      reply(Error::Busy);
      return;
    }
    movement_.abort();
    reply(Error::None);
    return;
  }
  if (!strcmp(cmd, "MOVEJ") || !strcmp(cmd, "SEG")) {
    bool segment = !strcmp(cmd, "SEG");
    if (n < 6) {
      reply(Error::BadArgs);
      return;
    }
    MoveRequest r;
    for (uint8_t i = 0; i < 4; ++i) {
      if (!number(t[i + 1], r.target.q[i])) {
        reply(Error::BadArgs);
        return;
      }
      if (i < 3)
        r.target.q[i] *= config::RAD_PER_DEG;
    }
    if (!number(t[5], r.T_wunsch)) {
      reply(Error::BadArgs);
      return;
    }
    bool gotV0 = false, gotA0 = false;
    uint8_t seen = 0;
    for (uint8_t p = 6; p < n;) {
      if (!strcmp(t[p], "EXACT")) {
        if (r.exact) {
          reply(Error::BadArgs);
          return;
        }
        r.exact = true;
        ++p;
        continue;
      }
      int8_t key = !strcmp(t[p], "V0")   ? 0
                   : !strcmp(t[p], "A0") ? 1
                   : !strcmp(t[p], "V1") ? 2
                   : !strcmp(t[p], "A1") ? 3
                                         : -1;
      if (key < 0 || p + 4 >= n || (seen & axisBit(uint8_t(key)))) {
        reply(Error::BadArgs);
        return;
      }
      seen |= axisBit(uint8_t(key));
      Joints* dest = key == 0   ? &r.startVelocity
                     : key == 1 ? &r.startAcceleration
                     : key == 2 ? &r.endVelocity
                                : &r.endAcceleration;
      for (uint8_t i = 0; i < 4; ++i) {
        if (!number(t[p + 1 + i], dest->q[i])) {
          reply(Error::BadArgs);
          return;
        }
        if (i < 3)
          dest->q[i] *= config::RAD_PER_DEG;
      }
      if (key == 0)
        gotV0 = true;
      if (key == 1)
        gotA0 = true;
      p += 5;
    }
    // V0 und A0 werden gemeinsam explizit geprüft; sonst aus dem vorherigen
    // Profil übernommen. Die erste Fahrt beginnt aus dem Stillstand.
    if (gotV0 != gotA0) {
      reply(Error::BadArgs);
      return;
    }
    r.explicitStart = gotV0;
    if (homing_.busy()) {
      reply(Error::Busy);
      return;
    }
    if (!segment) {
      Error e = ready(15);
      if (e != Error::None) {
        reply(e);
        return;
      }
      lastMove_ = Movement::Idle;
      reply(movement_.request(r));
    } else
      reply(movement_.append(r));
    return;
  }
  if (!strcmp(cmd, "SETJ") || !strcmp(cmd, "SETZ")) {
    bool joint = !strcmp(cmd, "SETJ");
    uint8_t base = joint ? 3 : 2;
    if (n != base && n != base + 1) {
      reply(Error::BadArgs);
      return;
    }
    int8_t id = joint ? axis(t[1]) : 3;
    if (id < 0 || (joint && id == 3)) {
      reply(Error::BadArgs);
      return;
    }
    Real target, T = config::DEFAULT_DURATION;
    if (!number(t[joint ? 2 : 1], target) || (n == base + 1 && !number(t[base], T))) {
      reply(Error::BadArgs);
      return;
    }
    uint8_t mask = axisBit(uint8_t(id));
    Error e = ready(mask);
    if (e != Error::None) {
      reply(e);
      return;
    }
    MoveRequest r;
    r.mask = mask;
    r.target = motors_.position();
    r.target.q[id] = joint ? target * config::RAD_PER_DEG : target;
    r.T_wunsch = T;
    lastMove_ = Movement::Idle;
    reply(movement_.request(r));
    return;
  }
  if (!strcmp(cmd, "MOVEXY")) {
    if (n < 3) {
      reply(Error::BadArgs);
      return;
    }
    Error e = ready(15);
    if (e != Error::None) {
      reply(e);
      return;
    }
    Pose p = kinematics_.forward(motors_.position());
    Real T = config::DEFAULT_DURATION;
    if (!number(t[1], p.x) || !number(t[2], p.y)) {
      reply(Error::BadArgs);
      return;
    }
    uint8_t seen = 0;
    for (uint8_t i = 3; i < n; i += 2) {
      if (i + 1 >= n) {
        reply(Error::BadArgs);
        return;
      }
      uint8_t key = !strcmp(t[i], "Z") ? 1 : !strcmp(t[i], "PSI") ? 2 : !strcmp(t[i], "T") ? 4 : 0;
      if (!key || (seen & key)) {
        reply(Error::BadArgs);
        return;
      }
      seen |= key;
      Real v;
      if (!number(t[i + 1], v)) {
        reply(Error::BadArgs);
        return;
      }
      if (key == 1)
        p.z = v;
      else if (key == 2)
        p.psi = v * config::RAD_PER_DEG;
      else
        T = v;
    }
    reply(movePose(p, T));
    return;
  }
  if (!strcmp(cmd, "SAVE") || !strcmp(cmd, "POSITION") || !strcmp(cmd, "DELETE")) {
    bool position = !strcmp(cmd, "POSITION");
    uint8_t id;
    if ((n != 2 && !(position && n == 3)) || !slot(t[1], id)) {
      reply(Error::BadArgs);
      return;
    }
    if (!strcmp(cmd, "DELETE")) {
      reply(store_.erase(id));
      return;
    }
    Error e = ready(15);
    if (e != Error::None) {
      reply(e);
      return;
    }
    if (!strcmp(cmd, "SAVE")) {
      reply(store_.save(id, kinematics_.forward(motors_.position())));
      return;
    }
    Pose p;
    e = store_.load(id, p);
    Real T = config::DEFAULT_DURATION;
    if (e != Error::None) {
      reply(e);
      return;
    }
    if (n == 3 && !number(t[2], T)) {
      reply(Error::BadArgs);
      return;
    }
    reply(movePose(p, T));
    return;
  }
  if (!strcmp(cmd, "LIST") && n == 1) {
    if (movement_.busy() || homing_.busy()) {
      reply(Error::Busy);
      return;
    }
    listCursor_ = 0;
    return;
  }
  if (movement_.busy() || homing_.busy()) {
    reply(Error::Busy);
    return;
  }
  if (!strcmp(cmd, "FK") && n == 5) {
    Joints q;
    for (uint8_t i = 0; i < 4; ++i) {
      if (!number(t[i + 1], q.q[i])) {
        reply(Error::BadArgs);
        return;
      }
      if (i < 3)
        q.q[i] *= config::RAD_PER_DEG;
    }
    pose(kinematics_.forward(q));
    return;
  }
  if ((!strcmp(cmd, "IK") || !strcmp(cmd, "REACH")) && n == 5) {
    Pose p;
    Real v[4];
    for (uint8_t i = 0; i < 4; ++i)
      if (!number(t[i + 1], v[i])) {
        reply(Error::BadArgs);
        return;
      }
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    p.psi = v[3] * config::RAD_PER_DEG;
    Joints q;
    Error e = kinematics_.inverse(p, motors_.position(), q);
    if (e != Error::None || !strcmp(cmd, "REACH")) {
      reply(e);
      return;
    }
    out_.print(F("IK J1/J2/J3_DEG,Z_MM: "));
    for (uint8_t i = 0; i < 4; ++i) {
      out_.print(i < 3 ? q.q[i] / config::RAD_PER_DEG : q.q[i], 3);
      out_.print(' ');
    }
    out_.println();
    return;
  }
  if (!strcmp(cmd, "COEFF") && n == 8) {
    Real v[7];
    for (uint8_t i = 0; i < 7; ++i)
      if (!number(t[i + 1], v[i])) {
        reply(Error::BadArgs);
        return;
      }
    QuinticProfile p;
    if (!p.configure(v[0], v[1], v[2], v[3], v[4], v[5], v[6])) {
      reply(Error::BadArgs);
      return;
    }
    for (uint8_t i = 0; i < 6; ++i) {
      out_.print(F("b"));
      out_.print(i);
      out_.print('=');
      out_.println(p.coefficients()[i], 6);
    }
    return;
  }
  if (!strcmp(cmd, "PROFILE") && (n == 3 || n == 9)) {
    Real T, tau, v[6] = {0, 0, 0, 1, 0, 0};
    if (!number(t[1], T) || !number(t[2], tau)) {
      reply(Error::BadArgs);
      return;
    }
    if (n == 9)
      for (uint8_t i = 0; i < 6; ++i)
        if (!number(t[i + 3], v[i])) {
          reply(Error::BadArgs);
          return;
        }
    QuinticProfile p;
    if (tau < 0 || tau > T || !p.configure(T, v[0], v[1], v[2], v[3], v[4], v[5])) {
      reply(Error::BadArgs);
      return;
    }
    Sample s = p.evaluate(tau);
    out_.print(F("q="));
    out_.print(s.position, 6);
    out_.print(F(" v="));
    out_.print(s.velocity, 6);
    out_.print(F(" a="));
    out_.print(s.acceleration, 6);
    out_.print(F(" jerk="));
    out_.println(s.jerk, 6);
    return;
  }
  reply(Error::BadArgs);
}
void SerialCommands::update() {
  out_.update();
  // LIST portionsweise ausgeben, damit auch 16 gefüllte Slots keinen TX-
  // Überlauf verursachen und STOP/Frames weiterhin regelmäßig bedient werden.
  if (listCursor_ < PositionStore::CAPACITY && out_.availableForWrite() >= 128) {
    uint8_t id = listCursor_++;
    Pose p;
    if (store_.load(id, p) == Error::None) {
      out_.print(F("POSITION "));
      out_.print(id);
      out_.print(' ');
      pose(p);
    }
  }
  uint8_t budget = 16;
  while (budget-- && Serial.available() > 0) {
    char c = char(Serial.read());
    if (c == '\r' || c == '\n') {
      if (discarding_) {
        discarding_ = false;
        length_ = 0;
        out_.println(F("ERR LINE_TOO_LONG"));
      } else if (length_) {
        buffer_[length_] = '\0';
        execute(buffer_);
        length_ = 0;
      }
      continue;
    }
    if (discarding_)
      continue;
    if ((c < ' ' && c != '\t') || length_ >= sizeof(buffer_) - 1) {
      discarding_ = true;
      length_ = 0;
      continue;
    }
    buffer_[length_++] = c;
  }
}
void SerialCommands::reportEvents() {
  MotorStatus m = motors_.status();
  if (m.error != lastFault_) {
    lastFault_ = m.error;
    if (m.error != Error::None) {
      out_.print(F("STOPPED "));
      out_.println(errorName(m.error));
    }
  }
  if (homing_.state() != lastHome_) {
    lastHome_ = homing_.state();
    if (lastHome_ == Homing::Done && m.error == Error::None)
      out_.println(F("DONE HOME"));
    else if (lastHome_ == Homing::Failed && m.error == Error::None)
      reply(homing_.error());
  }
  if (movement_.state() != lastMove_) {
    lastMove_ = movement_.state();
    if (lastMove_ == Movement::Running) {
      out_.print(F("PLAN T_fahrt="));
      out_.print(movement_.duration(), 4);
      out_.print(F(" segments="));
      out_.println(movement_.segmentCount());
    } else if (lastMove_ == Movement::Done && m.error == Error::None)
      out_.println(F("DONE MOVE"));
    else if (lastMove_ == Movement::Failed && m.error == Error::None)
      reply(movement_.error());
  }
}
}  // namespace scara
