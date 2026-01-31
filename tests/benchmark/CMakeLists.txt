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

add_executable(bm_jacobian dynamics/bm_jacobian.cpp)
target_link_libraries(bm_jacobian
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(dynamics/bm_jacobian.cpp)

# ==============================================================================
# Component Benchmarks (organized in subdirectories)
# ==============================================================================
add_subdirectory(lcpsolver)
if(DART_BUILD_SIMULATION_EXPERIMENTAL)
  add_subdirectory(simulation/experimental)
endif()

# ==============================================================================
# Math Benchmarks
# ==============================================================================
add_executable(bm_helpers math/bm_helpers.cpp)
target_link_libraries(bm_helpers
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(math/bm_helpers.cpp)

add_executable(bm_spatial_algebra math/bm_spatial_algebra.cpp)
target_link_libraries(bm_spatial_algebra
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
dart_format_add(math/bm_spatial_algebra.cpp)

# ==============================================================================
# SIMD Benchmarks
# ==============================================================================
add_executable(bm_simd simd/bm_simd.cpp)
target_link_libraries(bm_simd
  dart-simd
  Eigen3::Eigen
  benchmark::benchmark
)

# Enable native SIMD for benchmarks (AVX2, AVX-512, etc. based on CPU)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
  target_compile_options(bm_simd PRIVATE -march=native)
endif()

dart_format_add(simd/bm_simd.cpp)

# ==============================================================================
# Dynamics SIMD Benchmarks
# ==============================================================================
add_executable(bm_mesh_bbox dynamics/bm_mesh_bbox.cpp)
target_link_libraries(bm_mesh_bbox
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
target_link_libraries(bm_soft_body
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
