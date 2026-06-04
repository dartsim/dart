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
if(
  DART_BUILD_COLLISION_REFERENCE_BENCHMARKS
  AND TARGET dart-test-reference-bullet
)
  add_executable(bm_boxes collision/bm_boxes.cpp)
  target_link_libraries(
    bm_boxes
    dart
    dart-test-reference-bullet
    benchmark::benchmark
    benchmark::benchmark_main
  )
  dart_format_add(collision/bm_boxes.cpp)
endif()

add_executable(bm_native collision/bm_native.cpp)
target_link_libraries(bm_native dart-collision-native benchmark::benchmark)
dart_format_add(collision/bm_native.cpp)
add_executable(bm_native_ccd collision/native/bm_ccd.cpp)
target_link_libraries(bm_native_ccd dart-collision-native benchmark::benchmark)
dart_format_add(collision/native/bm_ccd.cpp)
if(DART_TESTS_LIBCCD_FOUND)
  add_executable(bm_native_libccd collision/native/bm_libccd.cpp)
  target_link_libraries(
    bm_native_libccd
    dart-collision-native
    dart_tests_libccd
    benchmark::benchmark
  )
  dart_format_add(collision/native/bm_libccd.cpp)
endif()
add_executable(bm_native_broadphase collision/native/bm_broadphase.cpp)
target_link_libraries(
  bm_native_broadphase
  dart-collision-native
  benchmark::benchmark
)
dart_format_add(collision/native/bm_broadphase.cpp)

if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS AND TARGET dart-test-reference-fcl)
  add_executable(bm_comparative collision/bm_comparative.cpp)
  target_link_libraries(
    bm_comparative
    dart
    dart-collision-native
    benchmark::benchmark
  )
  target_link_libraries(bm_comparative dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(bm_comparative dart-test-reference-bullet)
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(bm_comparative dart-test-reference-ode)
  endif()
  dart_format_add(collision/bm_comparative.cpp)
endif()

if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS AND TARGET dart-test-reference-fcl)
  add_executable(
    bm_comparative_narrow_phase
    collision/comparative/bm_narrow_phase.cpp
  )
  target_link_libraries(
    bm_comparative_narrow_phase
    dart
    dart-collision-native
    benchmark::benchmark
  )
  target_link_libraries(bm_comparative_narrow_phase dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(
      bm_comparative_narrow_phase
      dart-test-reference-bullet
    )
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(bm_comparative_narrow_phase dart-test-reference-ode)
  endif()
  dart_format_add(collision/comparative/bm_narrow_phase.cpp)
endif()

if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS AND TARGET dart-test-reference-fcl)
  add_executable(bm_comparative_distance collision/comparative/bm_distance.cpp)
  target_link_libraries(
    bm_comparative_distance
    dart
    dart-collision-native
    benchmark::benchmark
  )
  target_link_libraries(bm_comparative_distance dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(bm_comparative_distance dart-test-reference-bullet)
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(bm_comparative_distance dart-test-reference-ode)
  endif()
  dart_format_add(collision/comparative/bm_distance.cpp)
endif()
if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS)
  add_executable(bm_comparative_raycast collision/comparative/bm_raycast.cpp)
  target_link_libraries(
    bm_comparative_raycast
    dart
    dart-collision-native
    benchmark::benchmark
  )
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(bm_comparative_raycast dart-test-reference-bullet)
  endif()
  dart_format_add(collision/comparative/bm_raycast.cpp)
endif()
if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS AND TARGET dart-test-reference-fcl)
  add_executable(bm_comparative_ccd collision/comparative/bm_ccd.cpp)
  target_link_libraries(
    bm_comparative_ccd
    dart
    dart-collision-native
    benchmark::benchmark
  )
  target_link_libraries(bm_comparative_ccd dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(bm_comparative_ccd dart-test-reference-bullet)
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(bm_comparative_ccd dart-test-reference-ode)
  endif()
  dart_format_add(collision/comparative/bm_ccd.cpp)
endif()
if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS AND TARGET dart-test-reference-fcl)
  add_executable(
    bm_scenarios_mixed_primitives
    collision/scenarios/bm_mixed_primitives.cpp
  )
  target_link_libraries(
    bm_scenarios_mixed_primitives
    dart
    dart-collision-native
    benchmark::benchmark
  )
  target_link_libraries(bm_scenarios_mixed_primitives dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(
      bm_scenarios_mixed_primitives
      dart-test-reference-bullet
    )
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(bm_scenarios_mixed_primitives dart-test-reference-ode)
  endif()
  dart_format_add(collision/scenarios/bm_mixed_primitives.cpp)
endif()
add_executable(
  bm_scenarios_raycast_batch
  collision/scenarios/bm_raycast_batch.cpp
)
target_link_libraries(
  bm_scenarios_raycast_batch
  dart
  dart-collision-native
  benchmark::benchmark
)
if(
  DART_BUILD_COLLISION_REFERENCE_BENCHMARKS
  AND TARGET dart-test-reference-bullet
)
  target_link_libraries(bm_scenarios_raycast_batch dart-test-reference-bullet)
  target_compile_definitions(
    bm_scenarios_raycast_batch
    PRIVATE DART_ENABLE_COLLISION_REFERENCE_BENCHMARKS=1
  )
endif()
dart_format_add(collision/scenarios/bm_raycast_batch.cpp)
if(DART_BUILD_COLLISION_REFERENCE_BENCHMARKS AND TARGET dart-test-reference-fcl)
  add_executable(bm_scenarios_mesh_heavy collision/scenarios/bm_mesh_heavy.cpp)
  target_link_libraries(
    bm_scenarios_mesh_heavy
    dart
    dart-collision-native
    benchmark::benchmark
  )
  target_link_libraries(bm_scenarios_mesh_heavy dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(bm_scenarios_mesh_heavy dart-test-reference-bullet)
  endif()
  dart_format_add(collision/scenarios/bm_mesh_heavy.cpp)
endif()
add_executable(
  bm_scenarios_pipeline_breakdown
  collision/scenarios/bm_pipeline_breakdown.cpp
)
target_link_libraries(
  bm_scenarios_pipeline_breakdown
  dart
  dart-collision-native
  benchmark::benchmark
)
dart_format_add(collision/scenarios/bm_pipeline_breakdown.cpp)
add_executable(
  bm_scenarios_stacked_scenes
  collision/scenarios/bm_stacked_scenes.cpp
)
target_link_libraries(
  bm_scenarios_stacked_scenes
  dart
  dart-collision-native
  benchmark::benchmark
)
dart_format_add(collision/scenarios/bm_stacked_scenes.cpp)
add_executable(
  bm_scenarios_dart_adapter
  collision/scenarios/bm_dart_adapter.cpp
)
target_link_libraries(bm_scenarios_dart_adapter dart benchmark::benchmark)
dart_format_add(collision/scenarios/bm_dart_adapter.cpp)
dart_format_add(collision/fixtures/shape_factories.hpp)
dart_format_add(collision/fixtures/scene_builders.hpp)
dart_format_add(collision/fixtures/edge_cases.hpp)

# ==============================================================================
# Dynamics Benchmarks
# ==============================================================================
if(TARGET dart-io)
  add_executable(bm_kinematics dynamics/bm_kinematics.cpp)
  target_link_libraries(
    bm_kinematics
    dart-io
    benchmark::benchmark
    benchmark::benchmark_main
  )
  dart_format_add(dynamics/bm_kinematics.cpp)
endif()

add_executable(bm_jacobian dynamics/bm_jacobian.cpp)
target_link_libraries(
  bm_jacobian
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(dynamics/bm_jacobian.cpp)

add_executable(bm_dynamics_cache dynamics/bm_dynamics_cache.cpp)
target_link_libraries(
  bm_dynamics_cache
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(dynamics/bm_dynamics_cache.cpp)

add_executable(bm_sparse_occupancy_grid dynamics/bm_sparse_occupancy_grid.cpp)
target_link_libraries(
  bm_sparse_occupancy_grid
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
if(TARGET octomap)
  target_link_libraries(bm_sparse_occupancy_grid octomap)
  target_compile_definitions(
    bm_sparse_occupancy_grid
    PRIVATE DART_TESTS_HAVE_OCTOMAP=1
  )
endif()
dart_format_add(dynamics/bm_sparse_occupancy_grid.cpp)

if(TARGET dart-io)
  add_executable(bm_dynamics_cache_io dynamics/bm_dynamics_cache_io.cpp)
  target_link_libraries(
    bm_dynamics_cache_io
    dart-io
    benchmark::benchmark
    benchmark::benchmark_main
  )
  dart_format_add(dynamics/bm_dynamics_cache_io.cpp)
endif()

# ==============================================================================
# Common (Allocator) Benchmarks
# ==============================================================================
add_executable(bm_allocators common/bm_allocators.cpp)
target_link_libraries(
  bm_allocators
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(common/bm_allocators.cpp)

find_package(foonathan_memory QUIET)
if(foonathan_memory_FOUND)
  if(NOT TARGET EnTT::EnTT)
    dart_find_package(EnTT)
  endif()

  add_executable(bm_allocators_comparative common/bm_allocators_comparative.cpp)
  target_link_libraries(
    bm_allocators_comparative
    dart
    foonathan_memory
    benchmark::benchmark
  )
  if(TARGET EnTT::EnTT)
    target_link_libraries(bm_allocators_comparative EnTT::EnTT)
  endif()
  dart_format_add(common/bm_allocators_comparative.cpp)
endif()

# ==============================================================================
# Component Benchmarks (organized in subdirectories)
# ==============================================================================
add_subdirectory(lcpsolver)
# Guard on the target, not just the option: the experimental module skips itself
# in minimal configurations where its core sibling components (e.g. dart-io) are
# unavailable, leaving the option ON but the target absent.
if(DART_BUILD_SIMULATION_EXPERIMENTAL AND TARGET dart-simulation-experimental)
  add_subdirectory(simulation/experimental)
endif()

# ==============================================================================
# Math Benchmarks
# ==============================================================================
add_executable(bm_helpers math/bm_helpers.cpp)
target_link_libraries(
  bm_helpers
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(math/bm_helpers.cpp)

add_executable(bm_group_product math/bm_group_product.cpp)
target_link_libraries(
  bm_group_product
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(math/bm_group_product.cpp)

add_executable(bm_spatial_algebra math/bm_spatial_algebra.cpp)
target_link_libraries(
  bm_spatial_algebra
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(math/bm_spatial_algebra.cpp)

# ==============================================================================
# SIMD Benchmarks
# ==============================================================================
add_executable(bm_simd simd/bm_simd.cpp)
target_link_libraries(bm_simd dart-simd Eigen3::Eigen benchmark::benchmark)

add_executable(bm_math simd/bm_math.cpp)
target_link_libraries(bm_math dart-simd Eigen3::Eigen benchmark::benchmark)

# Enable native SIMD for benchmarks (AVX2, AVX-512, etc. based on CPU)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
  target_compile_options(bm_simd PRIVATE -march=native)
  target_compile_options(bm_math PRIVATE -march=native)
endif()

dart_format_add(simd/bm_simd.cpp)
dart_format_add(simd/bm_math.cpp)

# ==============================================================================
# Dynamics SIMD Benchmarks
# ==============================================================================
add_executable(bm_mesh_bbox dynamics/bm_mesh_bbox.cpp)
target_link_libraries(
  bm_mesh_bbox
  dart-simd
  Eigen3::Eigen
  benchmark::benchmark
  benchmark::benchmark_main
)
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE_MESH)
if(COMPILER_SUPPORTS_MARCH_NATIVE_MESH)
  target_compile_options(bm_mesh_bbox PRIVATE -march=native)
endif()
dart_format_add(dynamics/bm_mesh_bbox.cpp)

add_executable(bm_soft_body dynamics/bm_soft_body.cpp)
target_link_libraries(
  bm_soft_body
  dart-simd
  Eigen3::Eigen
  benchmark::benchmark
  benchmark::benchmark_main
)
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE_SOFT)
if(COMPILER_SUPPORTS_MARCH_NATIVE_SOFT)
  target_compile_options(bm_soft_body PRIVATE -march=native)
endif()
dart_format_add(dynamics/bm_soft_body.cpp)

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
