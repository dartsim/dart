#!/usr/bin/env bash

set -euo pipefail

task="${1:?usage: run_gz_physics_task.sh <build|install|test>}"

if [ -z "${DART_PARALLEL_JOBS:-}" ]; then
  if command -v nproc >/dev/null 2>&1; then
    DART_PARALLEL_JOBS="$(nproc)"
  else
    DART_PARALLEL_JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 2)"
  fi
fi
export DART_PARALLEL_JOBS

gz_physics_build_dir="${GZ_PHYSICS_BUILD_DIR:-.deps/gz-physics/build}"

case "$task" in
  build)
    cmake \
      --build "$gz_physics_build_dir" \
      --parallel "$DART_PARALLEL_JOBS" \
      --target all
    ;;
  install)
    for plugin in \
      dartsim-plugin \
      tpe-plugin \
      bullet-plugin \
      bullet-featherstone-plugin; do
      cmake \
        --build "$gz_physics_build_dir" \
        --parallel "$DART_PARALLEL_JOBS" \
        --target "gz-physics8-$plugin"
      for ext in so dylib; do
        versioned_plugin="lib/libgz-physics8-$plugin.$ext"
        unversioned_plugin="$gz_physics_build_dir/libgz-physics-$plugin.$ext"
        if [ -e "$gz_physics_build_dir/$versioned_plugin" ] \
          && [ ! -e "$unversioned_plugin" ]; then
          cmake -E create_symlink "$versioned_plugin" "$unversioned_plugin"
        fi
      done
    done
    cmake \
      --build "$gz_physics_build_dir" \
      --parallel "$DART_PARALLEL_JOBS" \
      --target install

    install_prefix="${GZ_PHYSICS_INSTALL_PREFIX:-}"
    if [ -z "$install_prefix" ]; then
      install_prefix="$(
        cmake -LA -N "$gz_physics_build_dir" \
          | sed -n 's/^CMAKE_INSTALL_PREFIX:PATH=//p' \
          | head -n 1
      )"
    fi

    engine_plugin_dir="$install_prefix/lib/gz-physics-8/engine-plugins"
    for plugin in \
      dartsim-plugin \
      tpe-plugin \
      bullet-plugin \
      bullet-featherstone-plugin; do
      for ext in so dylib; do
        versioned_plugin="libgz-physics8-$plugin.$ext"
        unversioned_plugin="libgz-physics-$plugin.$ext"
        if [ -e "$engine_plugin_dir/$versioned_plugin" ]; then
          cmake -E rm -f "$engine_plugin_dir/$unversioned_plugin"
          cmake -E create_symlink \
            "$versioned_plugin" \
            "$engine_plugin_dir/$unversioned_plugin"
        fi
      done
    done
    ;;
  test)
    export LD_LIBRARY_PATH="$gz_physics_build_dir/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="$gz_physics_build_dir/lib:$CONDA_PREFIX/lib:${DYLD_LIBRARY_PATH:-}"

    cd "$gz_physics_build_dir"
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
