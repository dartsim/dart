# Copyright (c) The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

if(DART_USE_SYSTEM_GOOGLEBENCHMARK)
  find_package(benchmark CONFIG REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.1
  )
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(benchmark)
endif()

include(dart_test_libccd)

# ==============================================================================
# Collision Benchmarks
# ==============================================================================
if(DART_BUILD_COLLISION_BULLET)
  add_executable(bm_boxes collision/bm_boxes.cpp)
  target_link_libraries(bm_boxes
    dart
    benchmark::benchmark
    benchmark::benchmark_main
  )
  dart_format_add(collision/bm_boxes.cpp)
endif()

if(DART_BUILD_COLLISION_EXPERIMENTAL)
  add_executable(bm_experimental collision/bm_experimental.cpp)
  target_link_libraries(bm_experimental
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/bm_experimental.cpp)
  add_executable(
    bm_experimental_ccd
    collision/experimental/bm_ccd.cpp
  )
  target_link_libraries(bm_experimental_ccd
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/experimental/bm_ccd.cpp)
  # TODO: bm_libccd.cpp doesn't exist yet - uncomment when created
  # if(DART_TESTS_LIBCCD_FOUND)
  #   add_executable(
  #     bm_experimental_libccd
  #     collision/experimental/bm_libccd.cpp
  #   )
  #   target_link_libraries(bm_experimental_libccd
  #     dart-collision-experimental
  #     dart_tests_libccd
  #     benchmark::benchmark
  #   )
  # endif()
  add_executable(
    bm_experimental_broadphase
    collision/experimental/bm_broadphase.cpp
  )
  target_link_libraries(bm_experimental_broadphase
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/experimental/bm_broadphase.cpp)

  add_executable(bm_comparative collision/bm_comparative.cpp)
  target_link_libraries(bm_comparative
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/bm_comparative.cpp)

  add_executable(
    bm_comparative_narrow_phase
    collision/comparative/bm_narrow_phase.cpp
  )
  target_link_libraries(bm_comparative_narrow_phase
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/comparative/bm_narrow_phase.cpp)
  add_executable(
    bm_comparative_distance
    collision/comparative/bm_distance.cpp
  )
  target_link_libraries(bm_comparative_distance
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/comparative/bm_distance.cpp)
  add_executable(
    bm_comparative_raycast
    collision/comparative/bm_raycast.cpp
  )
  target_link_libraries(bm_comparative_raycast
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/comparative/bm_raycast.cpp)
  add_executable(
    bm_scenarios_mixed_primitives
    collision/scenarios/bm_mixed_primitives.cpp
  )
  target_link_libraries(bm_scenarios_mixed_primitives
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/scenarios/bm_mixed_primitives.cpp)
  add_executable(
    bm_scenarios_raycast_batch
    collision/scenarios/bm_raycast_batch.cpp
  )
  target_link_libraries(bm_scenarios_raycast_batch
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/scenarios/bm_raycast_batch.cpp)
  add_executable(
    bm_scenarios_mesh_heavy
    collision/scenarios/bm_mesh_heavy.cpp
  )
  target_link_libraries(bm_scenarios_mesh_heavy
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/scenarios/bm_mesh_heavy.cpp)
  add_executable(
    bm_scenarios_pipeline_breakdown
    collision/scenarios/bm_pipeline_breakdown.cpp
  )
  target_link_libraries(bm_scenarios_pipeline_breakdown
    dart
    dart-collision-experimental
    benchmark::benchmark
  )
  dart_format_add(collision/scenarios/bm_pipeline_breakdown.cpp)
  dart_format_add(collision/fixtures/shape_factories.hpp)
  dart_format_add(collision/fixtures/scene_builders.hpp)
  dart_format_add(collision/fixtures/edge_cases.hpp)
endif()

# ==============================================================================
# Dynamics Benchmarks
# ==============================================================================
if(TARGET dart-io)
  add_executable(bm_kinematics dynamics/bm_kinematics.cpp)
  target_link_libraries(bm_kinematics
    dart-io
    benchmark::benchmark
    benchmark::benchmark_main
  )
  dart_format_add(dynamics/bm_kinematics.cpp)
endif()

# ==============================================================================
# Component Benchmarks (organized in subdirectories)
# ==============================================================================
add_subdirectory(lcpsolver)
if(DART_BUILD_SIMULATION_EXPERIMENTAL)
  add_subdirectory(simulation/experimental)
endif()

# ==============================================================================
# Note: Benchmarks are standalone executables for performance measurement.
# They are NOT registered as ctests since they measure performance, not correctness.
#
# Run benchmarks manually:
#   ./build/default/cpp/Release/tests/benchmark/bm_boxes
#   ./build/default/cpp/Release/tests/benchmark/bm_kinematics
#
# With custom settings:
#   ./bm_boxes --benchmark_min_time=1s --benchmark_repetitions=10
# ==============================================================================
