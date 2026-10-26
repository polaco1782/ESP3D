#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build=$(mktemp -d /tmp/esp3d-marlin-tests.XXXXXX)
trap 'rm -rf "$build"' EXIT HUP INT TERM
${CXX:-clang++} -std=c++11 -Wall -Wextra -Werror -pedantic -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$root/esp3d/src/modules/gcode_host" \
  "$root/esp3d/src/modules/gcode_host/marlin_stream.cpp" \
  "$root/tests/marlin_stream/protocol_test.cpp" -o "$build/protocol_test"
ASAN_OPTIONS=detect_leaks=0 "$build/protocol_test"
${CXX:-clang++} -std=c++11 -Wall -Wextra -Werror -Wno-unused-private-field \
  -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$root/tests/marlin_stream/stubs" -I"$root" \
  -I"$root/esp3d/src/modules/gcode_host" \
  -include "$root/tests/marlin_stream/stubs/host_environment.h" \
  "$root/esp3d/src/modules/gcode_host/gcode_host.cpp" \
  "$root/esp3d/src/modules/gcode_host/marlin_stream.cpp" \
  "$root/tests/marlin_stream/host_test.cpp" -o "$build/host_test"
ASAN_OPTIONS=detect_leaks=0 "$build/host_test"
