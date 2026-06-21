#!/usr/bin/env bash

set -euo pipefail

task="${1:?usage: run_gz_sim_task.sh <build|test>}"

if [ -z "${DART_PARALLEL_JOBS:-}" ]; then
  if command -v nproc >/dev/null 2>&1; then
    DART_PARALLEL_JOBS="$(nproc)"
  else
    DART_PARALLEL_JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 2)"
  fi
fi
export DART_PARALLEL_JOBS

repo_root="$PWD"
gz_sim_build_dir="$repo_root/.deps/gz-sim/build"
gz_physics_build_dir="${GZ_PHYSICS_BUILD_DIR:-$repo_root/.deps/gz-physics/build-install}"
gz_physics_prefix="${GZ_PHYSICS_INSTALL_PREFIX:-$repo_root/.deps/gz-physics/install}"
gz_physics_lib_dir="$gz_physics_prefix/lib"
if [ ! -d "$gz_physics_lib_dir/gz-physics-8/engine-plugins" ] \
  && [ -d "$gz_physics_prefix/lib64/gz-physics-8/engine-plugins" ]; then
  gz_physics_lib_dir="$gz_physics_prefix/lib64"
fi
gz_physics_engine_dir="$gz_physics_lib_dir/gz-physics-8/engine-plugins"
gz_physics_engine_path="$gz_physics_engine_dir"
if [ -e "$gz_physics_build_dir/libgz-physics-dartsim-plugin.so" ]; then
  gz_physics_engine_path="$gz_physics_build_dir:$gz_physics_engine_path"
fi
gz_physics_runtime_path="$gz_physics_lib_dir:$gz_physics_prefix/lib:$gz_physics_prefix/lib64"
if [ -d "$gz_physics_build_dir/lib" ]; then
  gz_physics_runtime_path="$gz_physics_build_dir/lib:$gz_physics_runtime_path"
fi

case "$task" in
  build)
    cmake \
      --build "$gz_sim_build_dir" \
      --parallel "$DART_PARALLEL_JOBS" \
      --target \
        gz-sim9-physics-system \
        gz-sim9-diff-drive-system \
        gz-sim9-scene-broadcaster-system \
        gz-sim9-sensors-system \
        gz-sim9-user-commands-system \
        gz-sim9-contact-system \
        INTEGRATION_entity_system
    ;;
  test)
    export LD_LIBRARY_PATH="$gz_sim_build_dir/lib:$gz_physics_runtime_path:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="$gz_sim_build_dir/lib:$gz_physics_runtime_path:$CONDA_PREFIX/lib:${DYLD_LIBRARY_PATH:-}"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$gz_sim_build_dir/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    export GZ_SIM_PHYSICS_ENGINE_PATH="$gz_physics_engine_path:${GZ_SIM_PHYSICS_ENGINE_PATH:-}"
    export GZ_CONFIG_PATH="$gz_sim_build_dir/test/conf:${GZ_CONFIG_PATH:-}"

    cd "$gz_sim_build_dir"
    ctest --output-on-failure --parallel 1 -R '^INTEGRATION_entity_system$'
    echo "gz-sim INTEGRATION_entity_system passed against the source-built DART plugin!"
    ;;
  *)
    echo "Unknown gz-sim task: $task" >&2
    exit 2
    ;;
esac
