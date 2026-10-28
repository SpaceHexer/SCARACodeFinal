#include "Types.h"

namespace scara {
const char* errorName(Error e) {
  switch (e) {
    case Error::None:
      return "NONE";
    case Error::BadArgs:
      return "BAD_ARGS";
    case Error::Busy:
      return "BUSY";
    case Error::NotHomed:
      return "NOT_HOMED";
    case Error::Stopped:
      return "STOPPED";
    case Error::ButtonHeld:
      return "STOP_BUTTON_HELD";
    case Error::NotConfigured:
      return "NOT_CONFIGURED";
    case Error::Unreachable:
      return "UNREACHABLE";
    case Error::Degenerate:
      return "IK_DEGENERATE";
    case Error::JointLimit:
      return "JOINT_LIMIT";
    case Error::ProfileLimit:
      return "PROFILE_LIMIT";
    case Error::Duration:
      return "DURATION_INFEASIBLE";
    case Error::Continuity:
      return "BOUNDARY_MISMATCH";
    case Error::NeedSuccessor:
      return "NEED_SUCCESSOR";
    case Error::Underrun:
      return "FRAME_UNDERRUN";
    case Error::Timing:
      return "TIMING_ERROR";
    case Error::StepRate:
      return "STEP_RATE_LIMIT";
    case Error::Endstop:
      return "ENDSTOP_ACTIVE";
    case Error::HomeTimeout:
      return "HOME_TIMEOUT";
    case Error::Storage:
      return "STORAGE_ERROR";
    case Error::EmptySlot:
      return "EMPTY_POSITION";
  }
  return "UNKNOWN";
}
}  // namespace scara
