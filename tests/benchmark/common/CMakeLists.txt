# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# This CMakeLists.txt file is for building the benchmarking programs for DART
set(link_dart_libraries
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
)

# Check if required DART libraries are built
foreach (link_dart_library ${link_dart_libraries})
  if(NOT TARGET ${link_dart_library})
    message(WARNING "DART benchmarking is disabled because ${link_dart_library} is not built")
    return()
  endif()
endforeach()

# Glob all the benchmark files
file(GLOB_RECURSE benchmark_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "benchmark_*.cpp")
if(benchmark_files)
  list(SORT benchmark_files)
endif()

set(link_libraries)

dart_build_benchmarks(
  TYPE ${benchmark_type}
  TEST_LIST integration_benchmarks
  SOURCES ${benchmark_files}
  LINK_LIBRARIES ${link_libraries}
  LINK_DART_LIBRARIES ${link_dart_libraries}
)
