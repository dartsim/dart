# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# DART 7.0 Benchmarks
#
# Simply list benchmark files here. The benchmark name is derived from filename.
# Example: bm_profiling.cpp -> bm_profiling
#
# Since benchmarks_dart7/ is a sibling to dart7/ (not a subdirectory), we need to
# include dependencies and helper functions here.

cmake_minimum_required(VERSION 3.22.1)

# Include dart7 utility functions (must come first to define dart7_find_package)
include(dart7_defs)

# Include dependency finding (makes benchmark available)
include(dart7_dependencies)

if(NOT DART7_BUILD_BENCHMARKS)
  message(STATUS "dart7: Benchmarks disabled (benchmark not found)")
  return()
endif()

message(STATUS "dart7: Configuring benchmarks...")

#==============================================================================
# Benchmark List
#==============================================================================
# To add a new benchmark, simply add the filename to this list

set(DART7_BENCHMARKS
  bm_profiling.cpp
)

# Register all benchmarks
foreach(bm_file ${DART7_BENCHMARKS})
  # Extract benchmark name from filename (remove .cpp extension)
  get_filename_component(bm_name ${bm_file} NAME_WE)
  dart7_add_benchmark(${bm_name} ${CMAKE_CURRENT_SOURCE_DIR}/${bm_file})
  list(APPEND DART7_BENCHMARK_TARGETS ${bm_name})
endforeach()

# Create meta target for all benchmarks
add_custom_target(dart7_benchmarks
  DEPENDS ${DART7_BENCHMARK_TARGETS}
  COMMENT "Building all DART7 benchmarks"
)

list(LENGTH DART7_BENCHMARKS num_benchmarks)
message(STATUS "dart7: ${num_benchmarks} benchmark(s)")
