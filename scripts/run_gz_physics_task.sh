#!/usr/bin/env bash

set -euo pipefail

task="${1:?usage: run_gz_physics_task.sh <build|test>}"

if [ -z "${DART_PARALLEL_JOBS:-}" ]; then
  if command -v nproc >/dev/null 2>&1; then
    DART_PARALLEL_JOBS="$(nproc)"
  else
    DART_PARALLEL_JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 2)"
  fi
fi
export DART_PARALLEL_JOBS

case "$task" in
  build)
    cmake \
      --build .deps/gz-physics/build \
      --parallel "$DART_PARALLEL_JOBS" \
      --target all
    ;;
  test)
    export LD_LIBRARY_PATH=".deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH=".deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${DYLD_LIBRARY_PATH:-}"

    cd .deps/gz-physics/build
    ctest --output-on-failure --parallel "$DART_PARALLEL_JOBS" -E PERFORMANCE_
    ctest --output-on-failure --parallel 1 -R PERFORMANCE_

    cd lib
    if ls libgz-physics8-dartsim-plugin*.dylib >/dev/null 2>&1; then
      otool -L libgz-physics8-dartsim-plugin*.dylib | grep dart
    else
      ldd libgz-physics8-dartsim-plugin.so | grep dart
    fi
    echo "Full gz-physics suite passed and DART plugin links successfully!"
    ;;
  *)
    echo "Unknown gz-physics task: $task" >&2
    exit 2
    ;;
esac
