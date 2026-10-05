#!/usr/bin/env bash
# Unpatched Gazebo compatibility lane: builds DART from a source tree, released
# gz-physics and gz-sim (no tools/gazebo/patches) against it, runs both full
# suites, and compares the failures with the lane's expected-failure file.
#
# usage: lane.sh <harmonic|ionic|jetty> [step] [args...]
#
# Steps (default: test):
#   test              dart, gz-physics, test-gz-physics, gz-sim, test-gz-sim,
#                     then compare (the gate)
#   dart              build DART and install it into the emptied variant prefix
#   gz-physics        build and install gz-physics with its tests
#   test-gz-physics   run the gz-physics suite (PERFORMANCE_ tests excluded)
#   gz-sim            build gz-sim with its tests and install it
#   fuel-cache        download the Fuel models gz-sim's tests use (once per
#                     work directory; test-gz-sim runs it)
#   test-gz-sim       run the full gz-sim INTEGRATION suite serially
#   compare           compare the results with <lane>-expected-failures.txt
#                     (and with the base variant's, see GZ_COMPAT_BASE_VARIANT)
#   baseline          rewrite <lane>-expected-failures.txt from the results
#   bench             build the drivers in tools/gazebo/bench
#   worlds            write the benchmark worlds derived from 3k_shapes.sdf
#   bench-gz-physics  gz-physics stepping benchmark; args go to the driver
#                     (default world: gz-sim's 3k_shapes.sdf, 3000 steps)
#   bench-gz-sim      gz-sim server benchmark; args go to the driver
#                     (default: 3k_shapes with real-time factor 0)
#   sleep-oracle      differential sleep oracle; args go to the driver
#   raycast-probe     single and batched rays over DART's ODE and Bullet
#                     collision groups (jetty only); args go to the driver
#
# Environment:
#   GZ_COMPAT_DIR          work directory (default: .deps/gz-compat/<lane>)
#   GZ_COMPAT_DART_SOURCE  DART source tree (default: this checkout)
#   GZ_COMPAT_VARIANT      name of the DART build in the work directory
#                          (default: candidate); each variant gets its own
#                          gz-physics and gz-sim builds
#   GZ_COMPAT_BASE_VARIANT variant holding the candidate's base (for example
#                          the release branch before the change), never the
#                          candidate's own; compare then reports failures
#                          the base also has instead of failing on them
#   DART_PARALLEL_JOBS     build jobs (default: nproc)
#   GZ_COMPAT_TEST_JOBS    parallel gz-physics tests (default: build jobs)
#
# gz-cmake gives every gz-physics and gz-sim GoogleTest test a 240 s TIMEOUT
# property, which `ctest --timeout` cannot raise.
set -euo pipefail
# An inherited GTEST_FILTER, sharding or GTEST_FAIL_FAST would run only part of
# the gz-physics and gz-sim suites, which compare cannot always tell from a
# full run.
unset "${!GTEST_@}"

usage() {
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'
}

lane="${1:-}"
step="${2:-test}"
shift $(($# < 2 ? $# : 2))

case "$lane" in
  harmonic)
    gz_physics_ref=gz-physics7_7.8.0
    gz_sim_ref=gz-sim8_8.10.0
    gz_physics_major=7
    # With conda-forge's GCC 15: gz-common5 headers use uint32_t without
    # <cstdint>, and gz-sim 8.10.0's reset_sensors test has a template member
    # GCC 15 rejects (-Wtemplate-body). Only the Gazebo builds get these flags,
    # so the DART build still catches a missing include in DART's headers.
    gz_cxx_flags="-include cstdint -Wno-template-body"
    ;;
  ionic)
    gz_physics_ref=gz-physics8_8.4.0
    # gz-sim 9.6.0 needs a newer gz-common6 than conda-forge ships.
    gz_sim_ref=gz-sim9_9.5.0
    gz_physics_major=8
    ;;
  jetty)
    gz_physics_ref=gz-physics9_9.5.2
    gz_sim_ref=gz-sim10_10.5.0
    gz_physics_major=9
    ;;
  -h | --help | "")
    usage
    exit 0
    ;;
  *)
    echo "Unknown lane: $lane (expected harmonic, ionic or jetty)" >&2
    exit 2
    ;;
esac

: "${CONDA_PREFIX:?run lane.sh through pixi (pixi run gz-compat-$lane)}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
compat_dir="$repo_root/tools/gazebo/compat"
variant="${GZ_COMPAT_VARIANT:-candidate}"
base_variant="${GZ_COMPAT_BASE_VARIANT:-}"
# Variants name directories under the work directory, and a candidate's DART
# prefix is deleted before each install.
for name in "$variant" "$base_variant"; do
  case "$name" in
    */* | . | ..)
      echo "Variant names must be plain names: $name" >&2
      exit 2
      ;;
  esac
done
# compare would read the candidate's results as the base's and report every
# new failure as a failure the base also has.
if [ -n "$base_variant" ] && [ "$base_variant" = "$variant" ]; then
  echo "GZ_COMPAT_BASE_VARIANT names the candidate variant ($variant);" \
    "set it to the variant built from the base" >&2
  exit 2
fi
# After the name checks, which test-gz-compat-tools runs on every platform:
# `realpath -m` is GNU-only.
work_dir="$(realpath -m "${GZ_COMPAT_DIR:-$repo_root/.deps/gz-compat/$lane}")"
dart_source="$(realpath "${GZ_COMPAT_DART_SOURCE:-$repo_root}")"
jobs="${DART_PARALLEL_JOBS:-$(nproc)}"
test_jobs="${GZ_COMPAT_TEST_JOBS:-$jobs}"

src_dir="$work_dir/src"
variant_dir="$work_dir/$variant"
dart_prefix="$variant_dir/dart"
gz_physics_build="$variant_dir/gz-physics-build"
gz_physics_prefix="$variant_dir/gz-physics"
gz_sim_build="$variant_dir/gz-sim-build"
gz_sim_prefix="$variant_dir/gz-sim"
bench_build="$variant_dir/bench-build"
results_dir="$variant_dir/results"
worlds_dir="$work_dir/worlds"
fuel_cache="$work_dir/fuel-cache"
engine_dir="$gz_physics_prefix/lib/gz-physics-$gz_physics_major/engine-plugins"
dartsim_plugin="$engine_dir/libgz-physics-dartsim-plugin.so"
expected="$compat_dir/$lane-expected-failures.txt"
# Included at the top-level project() of the gz-physics and gz-sim builds.
gz_top_level_includes="$repo_root/cmake/gz_force_vendor_gtest.cmake;$compat_dir/order_check_tests.cmake"

log() {
  echo "[gz-compat $lane/$variant] $*"
}

# Runs a Gazebo configure or build. CXXFLAGS (not only CMAKE_CXX_FLAGS) also
# reaches gz-physics' nested test project, which configures at build time.
gz_build_env() {
  CXXFLAGS="${CXXFLAGS:-} ${gz_cxx_flags:-}" "$@"
}

runtime_library_path() {
  echo "$dart_prefix/lib:$gz_physics_prefix/lib:$gz_sim_prefix/lib:$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
}

clone() (
  local name=$1 ref=$2
  mkdir -p "$src_dir"
  # Variants share released sources. Hold the lock through validation so a
  # concurrent variant cannot inspect a clone that is still being populated.
  exec 9> "$work_dir/clone.lock"
  flock 9
  if [ ! -d "$src_dir/$name/.git" ]; then
    log "cloning $name $ref"
    git clone --quiet --depth 1 --branch "$ref" \
      "https://github.com/gazebosim/$name" "$src_dir/$name"
  fi
  local actual
  actual="$(git -C "$src_dir/$name" describe --tags --exact-match 2>/dev/null || true)"
  if [ "$actual" != "$ref" ]; then
    echo "$src_dir/$name is at '${actual:-unknown}', expected $ref" >&2
    exit 1
  fi
  # The tag still matches after local edits, which the lane would build and
  # report as the released, unpatched sources. An untracked dartsim/src/*.cc
  # is built too, so list untracked files whatever status.showUntrackedFiles
  # says.
  if [ -n "$(git -C "$src_dir/$name" status --porcelain --untracked-files=normal)" ]; then
    echo "$src_dir/$name has local changes; restore $ref with" \
      "'git -C $src_dir/$name reset --hard && git -C $src_dir/$name clean -fd'" \
      "or delete the directory to clone it again" >&2
    exit 1
  fi
)

step_dart() {
  log "building DART from $dart_source"
  # The DART_SKIP_* options only exist in DART 6.19 (for the 6.19.4 baselines):
  # they keep its optional optimizer plugins, which gz-physics does not use,
  # from picking up host packages outside the environment. CMAKE_CXX_FLAGS is
  # passed explicitly so the build never keeps flags cached by older runs.
  cmake -G Ninja -S "$dart_source" -B "$variant_dir/dart-build" \
    --no-warn-unused-cli \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="${CXXFLAGS:-}" \
    -DCMAKE_INSTALL_PREFIX="$dart_prefix" \
    -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
    -DBUILD_TESTING=OFF \
    -DDART_BUILD_DARTPY=OFF \
    -DDART_BUILD_GUI_OSG=OFF \
    -DDART_BUILD_PROFILE=OFF \
    -DDART_TREAT_WARNINGS_AS_ERRORS=OFF \
    -DDART_SKIP_IPOPT=ON \
    -DDART_SKIP_NLOPT=ON \
    -DDART_SKIP_pagmo=ON
  # Install into an empty prefix: a reused one keeps the headers, libraries
  # and CMake files an earlier candidate installed and this one does not,
  # which gz-physics would find. Only DART installs there; the build
  # directory stays for incremental builds.
  rm -rf "$dart_prefix"
  cmake --build "$variant_dir/dart-build" --parallel "$jobs" --target install
}

# Configures from inside the build directory: gz-physics 7 and 8 create their
# unversioned plugin symlinks in the working directory at configure time.
configure() {
  local build=$1
  shift
  mkdir -p "$build"
  (cd "$build" && cmake -G Ninja -B . "$@")
}

step_gz_physics() {
  clone gz-physics "$gz_physics_ref"
  log "building $gz_physics_ref against $dart_prefix"
  # The common tests only get the dartsim plugin's DART_HAS_CONTACT_SURFACE
  # definition once its include check is cached, so a single configure drops
  # their contact-callback expectations. Configure twice so they always run.
  local pass
  for pass in 1 2; do
    gz_build_env configure "$gz_physics_build" -S "$src_dir/gz-physics" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$gz_physics_prefix" \
      -DCMAKE_PREFIX_PATH="$dart_prefix;$CONDA_PREFIX" \
      -DCMAKE_INSTALL_RPATH="$gz_physics_prefix/lib;$dart_prefix/lib;$CONDA_PREFIX/lib" \
      -DCMAKE_CXX_FLAGS="${CXXFLAGS:-} ${gz_cxx_flags:-} -I$src_dir/gz-physics/test/gtest_vendor/include" \
      -DCMAKE_PROJECT_TOP_LEVEL_INCLUDES:STRING="$gz_top_level_includes" \
      -DBUILD_TESTING=ON > /dev/null
  done
  gz_build_env cmake --build "$gz_physics_build" --parallel "$jobs"
  gz_build_env cmake --build "$gz_physics_build" --parallel "$jobs" --target install
}

step_test_gz_physics() {
  # A build configured before order_check_tests.cmake existed lacks the check_
  # ordering, so reconfigure it.
  grep -q order_check_tests "$gz_physics_build/CMakeCache.txt" 2> /dev/null ||
    step_gz_physics
  mkdir -p "$results_dir"
  # CTest writes the JUnit report only when it finishes, so an interrupted run
  # leaves none (and compare stops) instead of the previous run's.
  rm -rf "$gz_physics_build/test_results" "$results_dir/gz-physics-gtest" \
    "$results_dir/gz-physics.junit.xml"
  log "running the gz-physics suite"
  local status=0
  LD_LIBRARY_PATH="$(runtime_library_path)" \
    ctest --test-dir "$gz_physics_build" --output-on-failure \
    --parallel "$test_jobs" -E PERFORMANCE_ \
    --output-junit "$results_dir/gz-physics.junit.xml" \
    > "$results_dir/gz-physics.log" 2>&1 || status=$?
  cp -r "$gz_physics_build/test_results" "$results_dir/gz-physics-gtest"
  tail -n 3 "$results_dir/gz-physics.log"
  log "gz-physics ctest exit $status (failures are judged by compare)"
}

step_gz_sim() {
  clone gz-sim "$gz_sim_ref"
  if [ ! -d "$gz_physics_prefix" ]; then
    step_gz_physics
  fi
  log "building $gz_sim_ref with tests"
  # gz-sim finds the dartsim plugin through the gz-physics it was built
  # against (and conda-forge links with RPATH, which LD_LIBRARY_PATH does not
  # override), so every variant needs its own gz-sim build.
  gz_build_env configure "$gz_sim_build" -S "$src_dir/gz-sim" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$gz_sim_prefix" \
    -DCMAKE_PREFIX_PATH="$gz_physics_prefix;$CONDA_PREFIX" \
    -DCMAKE_INSTALL_RPATH="$gz_sim_prefix/lib;$gz_physics_prefix/lib;$CONDA_PREFIX/lib" \
    -DCMAKE_CXX_FLAGS="${CXXFLAGS:-} ${gz_cxx_flags:-} -I$src_dir/gz-sim/test/gtest_vendor/include" \
    -DCMAKE_PROJECT_TOP_LEVEL_INCLUDES:STRING="$gz_top_level_includes" \
    -DSKIP_PYBIND11=ON \
    -DBUILD_TESTING=ON
  gz_build_env cmake --build "$gz_sim_build" --parallel "$jobs"
  gz_build_env cmake --build "$gz_sim_build" --parallel "$jobs" --target install
}

# gz-sim's test fixture gives each test case a fake home and deletes it
# afterwards, so every case downloaded its Fuel models again and unpacked them
# in the test process. In test binaries that load assimp before libzip (on
# Ionic, the model_photo_shoot tests), assimp's bundled zip_open takes the
# place of libzip's: unpacking fails, or crashes in libzip. This step downloads
# every Fuel model gz-sim's tests name, in a `gz fuel` process, checks that
# each one unpacked, and publishes the cache that every variant of the work
# directory then reads. The models come from the tests and from the example
# worlds the tests load; the cache is rebuilt when that list changes. Delete it
# to download the models again.
fuel_model_uris() {
  local worlds=()
  mapfile -t worlds < <(
    {
      grep -rhoE 'examples/worlds/[A-Za-z0-9_.-]+\.sdf' "$src_dir/gz-sim/test"
      grep -rhoE '"examples",[[:space:]]*"worlds",[[:space:]]*"[A-Za-z0-9_.-]+\.sdf"' \
        "$src_dir/gz-sim/test" | sed -E 's/.*"([A-Za-z0-9_.-]+\.sdf)"$/examples\/worlds\/\1/'
    } | sort -u | sed "s|^|$src_dir/gz-sim/|")
  grep -rhIoE 'https://fuel\.gazebosim\.org/1\.0/[^/<"]+/models/[^/<"]+(/[0-9]+)?' \
    "$src_dir/gz-sim/test" "${worlds[@]}" | sed 's/[[:space:]]*$//' | sort -u
}

step_fuel_cache() {
  clone gz-sim "$gz_sim_ref"
  local uris
  uris="$(fuel_model_uris)"
  [ -f "$fuel_cache/.uris" ] && [ "$(cat "$fuel_cache/.uris")" = "$uris" ] && return
  log "downloading the Fuel models gz-sim's tests use into $fuel_cache"
  local staging old='' uri path owner name version models
  staging="$(mktemp -d "$fuel_cache.XXXXXX")"
  while IFS= read -r uri; do
    path="${uri#*/1.0/}" # <owner>/models/<name>[/<version>]
    owner="${path%%/*}"
    name="${path#*/models/}"
    version="${name#*/}"
    [ "$version" != "$name" ] || version='*'
    name="${name%%/*}"
    # `gz fuel download` exits 0 even when the download fails.
    HOME="$staging/.home" GZ_FUEL_CACHE_PATH="$staging" \
      gz fuel download -u "$uri" > /dev/null 2>&1 || true
    models=("$staging/fuel.gazebosim.org/${owner,,}/models/${name,,}/"$version/model.config)
    if [ ! -f "${models[0]}" ] || [ -n "$(find "$staging" -name '*.zip' -print -quit)" ]; then
      echo "error: could not download and unpack $uri" >&2
      rm -rf "$staging"
      exit 1
    fi
  done <<< "$uris"
  rm -rf "$staging/.home"
  printf '%s\n' "$uris" > "$staging/.uris"
  # Replace an outdated cache. Test runs read their own copy (see
  # step_test_gz_sim), and a concurrent run may have published first.
  if [ -d "$fuel_cache" ]; then
    old="$(mktemp -d "$fuel_cache.old.XXXXXX")"
    mv -T "$fuel_cache" "$old/cache" 2> /dev/null || true
  fi
  mv -T "$staging" "$fuel_cache" 2> /dev/null || rm -rf "$staging"
  [ -z "$old" ] || rm -rf "$old"
}

step_test_gz_sim() {
  [ -d "$gz_sim_prefix" ] || step_gz_sim
  step_fuel_cache
  mkdir -p "$results_dir"
  rm -rf "$gz_sim_build/test_results" "$results_dir/gz-sim-gtest" \
    "$results_dir/gz-sim.junit.xml" "$results_dir/gz-sim-tmp"
  mkdir -p "$results_dir/gz-sim-tmp"
  # The tests read a per-run copy of the cache, so a test that still downloads
  # (one that sets its own resource cache) cannot change the shared one.
  rm -rf "$results_dir/gz-sim-fuel"
  cp -al "$fuel_cache" "$results_dir/gz-sim-fuel" 2> /dev/null ||
    cp -a "$fuel_cache" "$results_dir/gz-sim-fuel"
  log "running the gz-sim INTEGRATION suite with $dartsim_plugin"
  local status=0
  # Without GZ_SIM_SERVER_CONFIG_PATH, test worlds that declare no systems get
  # no Physics system and many physics tests silently test nothing. The tests
  # keep a fake home under the temporary directory and delete it after each
  # case, so a private TMPDIR (and transport partition) keeps concurrent runs
  # on one host apart. With GZ_FUEL_CACHE_PATH the tests read their Fuel
  # models from the lane's cache instead of downloading them; tests that set
  # their own resource cache still download. (log_system's LogResources case
  # looks for its model under the fake home, but log_system crashes before it
  # on DART 6.19.4.) Some gz-sim tests are timing-sensitive, so a test that
  # fails, then passes on its retry, counts as passing (compare lists it as
  # FLAKY) unless an attempt crashed or timed out; compare reads every attempt
  # from the log, because the JUnit report and the XML keep only the last one.
  # gz-cmake's check_ tests record a failure for a test that left no results,
  # for example after a crash.
  LD_LIBRARY_PATH="$(runtime_library_path)" \
    TMPDIR="$results_dir/gz-sim-tmp" \
    GZ_SIM_SERVER_CONFIG_PATH="$src_dir/gz-sim/include/gz/sim/server.config" \
    GZ_SIM_SYSTEM_PLUGIN_PATH="$gz_sim_build/lib" \
    GZ_SIM_PHYSICS_ENGINE_PATH="$engine_dir" \
    GZ_CONFIG_PATH="$gz_sim_build/test/conf" \
    GZ_FUEL_CACHE_PATH="$results_dir/gz-sim-fuel" \
    GZ_IP=127.0.0.1 \
    GZ_PARTITION="gz-compat-$lane-$variant-$$" \
    ctest --test-dir "$gz_sim_build" --output-on-failure \
    --parallel 1 -R '^(check_)?INTEGRATION_' \
    --repeat until-pass:2 \
    --output-junit "$results_dir/gz-sim.junit.xml" \
    > "$results_dir/gz-sim.log" 2>&1 || status=$?
  cp -r "$gz_sim_build/test_results" "$results_dir/gz-sim-gtest"
  tail -n 3 "$results_dir/gz-sim.log"
  log "gz-sim ctest exit $status (failures are judged by compare)"
}

step_compare() {
  local base=()
  if [ -n "$base_variant" ]; then
    base=(--base-results "$work_dir/$base_variant/results")
  fi
  python "$compat_dir/compare_failures.py" \
    --results "$results_dir" --expected "$expected" ${base[@]+"${base[@]}"} "$@"
}

step_baseline() {
  local dart_version
  dart_version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' \
    "$dart_source/package.xml")"
  if [ -e "$dart_source/.git" ]; then
    dart_version+=" ($(git -C "$dart_source" describe --tags --always --dirty))"
  fi
  step_compare --write-baseline \
    --describe "$lane lane: $gz_physics_ref + $gz_sim_ref, DART $dart_version" \
    "$@"
}

step_bench() {
  if [ ! -d "$gz_sim_prefix" ]; then
    step_gz_sim
  fi
  gz_build_env cmake -G Ninja -S "$repo_root/tools/gazebo/bench" -B "$bench_build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$dart_prefix;$gz_physics_prefix;$gz_sim_prefix;$CONDA_PREFIX" \
    -DCMAKE_INSTALL_RPATH="$dart_prefix/lib;$gz_physics_prefix/lib;$gz_sim_prefix/lib;$CONDA_PREFIX/lib" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DGZ_RELEASE="$lane" \
    -DGZ_PHYSICS_SOURCE_DIR="$src_dir/gz-physics"
  gz_build_env cmake --build "$bench_build" --parallel "$jobs"
}

world_3k="$src_dir/gz-sim/examples/worlds/3k_shapes.sdf"

step_worlds() {
  clone gz-sim "$gz_sim_ref"
  python "$repo_root/tools/gazebo/bench/make_worlds.py" "$world_3k" "$worlds_dir"
}

# The driver steps rebuild the drivers first (a no-op when they are current),
# so they never run a stale binary; build output goes to stderr.
step_bench_gz_physics() {
  step_bench >&2
  if [ $# -eq 0 ]; then
    set -- "$world_3k" 3000 --window 250 --sunk-z 0.45
  fi
  LD_LIBRARY_PATH="$(runtime_library_path)" GZ_SIM_RESOURCE_PATH="" \
    "$bench_build/gz_physics_step_bench" "$dartsim_plugin" "$@"
}

step_bench_gz_sim() {
  step_bench >&2
  if [ $# -eq 0 ]; then
    # The server only steps as fast as the world's real-time factor allows.
    [ -f "$worlds_dir/3k_shapes.sdf" ] || step_worlds
    set -- "$worlds_dir/3k_shapes.sdf" 1000 250
  fi
  LD_LIBRARY_PATH="$(runtime_library_path)" \
    GZ_SIM_SERVER_CONFIG_PATH="$src_dir/gz-sim/include/gz/sim/server.config" \
    GZ_SIM_PHYSICS_ENGINE_PATH="$engine_dir" \
    GZ_IP=127.0.0.1 GZ_PARTITION="gz-compat-bench-$$" GZ_SIM_RESOURCE_PATH="" \
    "$bench_build/gz_sim_server_bench" --engine "$dartsim_plugin" "$@"
}

step_sleep_oracle() {
  step_bench >&2
  LD_LIBRARY_PATH="$(runtime_library_path)" \
    "$bench_build/gz_sleep_oracle" "$dartsim_plugin" "$@"
}

step_raycast_probe() {
  if [ "$gz_physics_major" -lt 9 ]; then
    echo "raycast-probe needs gz-physics 9 (the jetty lane)" >&2
    exit 2
  fi
  step_bench >&2
  LD_LIBRARY_PATH="$(runtime_library_path)" \
    "$bench_build/gz_raycast_probe" "$dartsim_plugin" "$@"
}

case "$step" in
  test)
    step_dart
    step_gz_physics
    step_test_gz_physics
    step_gz_sim
    step_test_gz_sim
    step_compare "$@"
    ;;
  dart) step_dart ;;
  gz-physics) step_gz_physics ;;
  test-gz-physics) step_test_gz_physics ;;
  gz-sim) step_gz_sim ;;
  fuel-cache) step_fuel_cache ;;
  test-gz-sim) step_test_gz_sim ;;
  compare) step_compare "$@" ;;
  baseline) step_baseline "$@" ;;
  bench) step_bench ;;
  worlds) step_worlds ;;
  bench-gz-physics) step_bench_gz_physics "$@" ;;
  bench-gz-sim) step_bench_gz_sim "$@" ;;
  sleep-oracle) step_sleep_oracle "$@" ;;
  raycast-probe) step_raycast_probe "$@" ;;
  *)
    echo "Unknown step: $step" >&2
    usage >&2
    exit 2
    ;;
esac
