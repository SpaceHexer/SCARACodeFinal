#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
scara_timing_binary=$(mktemp /tmp/scara-avr-timing.XXXXXX)
trap 'rm -f "$scara_timing_binary"' EXIT
# Linux: libsimavr-dev installieren. Optional entpackte Pakete via Prefix nutzen.
scara_simavr_prefix=${SIMAVR_PREFIX:-/usr}
scara_simavr_lib="$scara_simavr_prefix/lib/$(gcc -print-multiarch)"
gcc -std=c99 -Wall -Wextra -Werror \
  -I"$scara_simavr_prefix/include/simavr" tests/avr-timing/runner.c \
  -L"$scara_simavr_lib" -Wl,-rpath,"$scara_simavr_lib" -lsimavr -o "$scara_timing_binary"
"$scara_timing_binary" "${1:-.pio/build/megaatmega2560/firmware.elf}"
