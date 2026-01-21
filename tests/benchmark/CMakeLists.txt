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
# SIMD Benchmarks
# ==============================================================================
add_executable(bm_simd simd/bm_simd.cpp)
target_link_libraries(bm_simd
  dart-simd
  benchmark::benchmark
)
dart_format_add(simd/bm_simd.cpp)

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
