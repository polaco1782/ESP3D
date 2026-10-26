#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build=$(mktemp -d /tmp/esp3d-worker-tests.XXXXXX)
trap 'rm -rf "$build"' EXIT HUP INT TERM
${CXX:-clang++} -std=c++11 -pthread -Wall -Wextra -Werror -Wno-unused-private-field \
  -g -fsanitize="${SANITIZERS:-address,undefined}" -fno-omit-frame-pointer \
  -DTEST_MARLIN_WORKER -DTEST_MAIN_CORE="${MAIN_CORE:-0}" -DCONFIG_FREERTOS_UNICORE="${UNICORE:-0}" \
  -I"$root/tests/marlin_stream/stubs" -I"$root" \
  -I"$root/esp3d/src/modules/gcode_host" \
  -include "$root/tests/marlin_stream/stubs/host_environment.h" \
  "$root/esp3d/src/modules/gcode_host/gcode_host.cpp" \
  "$root/esp3d/src/modules/gcode_host/gcode_host_worker.cpp" \
  "$root/esp3d/src/modules/gcode_host/marlin_stream.cpp" \
  "$root/tests/marlin_stream/freertos_test_runtime.cpp" \
  "$root/tests/marlin_stream/worker_test.cpp" -o "$build/worker_test"
ASAN_OPTIONS=detect_leaks=0 "$build/worker_test"
