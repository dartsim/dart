# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# DART 8.0 Benchmarks
#
# Simply list benchmark files here. The benchmark name is derived from filename.
# Example: bm_profiling.cpp -> bm_profiling
#
# Since benchmarks_dart8/ is a sibling to dart8/ (not a subdirectory), we need to
# include dependencies and helper functions here.

cmake_minimum_required(VERSION 3.22.1)

# Include dart8 utility functions (must come first to define dart8_find_package)
include(dart8_defs)

# Include dependency finding (makes benchmark available)
include(dart8_dependencies)

if(NOT DART8_BUILD_BENCHMARKS)
  message(STATUS "dart8: Benchmarks disabled (benchmark not found)")
  return()
endif()

message(STATUS "dart8: Configuring benchmarks...")

#==============================================================================
# Benchmark List
#==============================================================================
# To add a new benchmark, simply add the filename to this list

set(DART8_BENCHMARKS
  bm_profiling.cpp
)

# Register all benchmarks
foreach(bm_file ${DART8_BENCHMARKS})
  # Extract benchmark name from filename (remove .cpp extension)
  get_filename_component(bm_name ${bm_file} NAME_WE)
  dart8_add_benchmark(${bm_name} ${CMAKE_CURRENT_SOURCE_DIR}/${bm_file})
  list(APPEND DART8_BENCHMARK_TARGETS ${bm_name})
endforeach()

# Create meta target for all benchmarks
add_custom_target(dart8_benchmarks
  DEPENDS ${DART8_BENCHMARK_TARGETS}
  COMMENT "Building all DART8 benchmarks"
)

list(LENGTH DART8_BENCHMARKS num_benchmarks)
message(STATUS "dart8: ${num_benchmarks} benchmark(s)")
