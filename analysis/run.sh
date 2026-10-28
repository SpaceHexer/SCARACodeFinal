#!/usr/bin/env bash
set -euo pipefail
scara_analysis_dir=$(cd "$(dirname "$0")" && pwd)
scara_source_dir=${1:-/workspace/SCARA-Roboter-Serial/SCARA-Roboter_PlattformIO}
scara_binary=$(mktemp /tmp/scara-review.XXXXXX)
trap 'rm -f "$scara_binary"' EXIT
cd "$scara_source_dir"
g++ -std=c++11 -Itests/host -Iinclude -x c++ "$scara_analysis_dir/archive/probe-legacy.cpp.txt" \
  src/Types.cpp src/Kinematics.cpp src/QuinticProfile.cpp src/MotorController.cpp \
  src/Homing.cpp src/Movement.cpp src/SerialCommands.cpp -o "$scara_binary"
"$scara_binary"
