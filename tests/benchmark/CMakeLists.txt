# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# GoogleBenchmark
if(DART_DOWNLOAD_DEPENDENT_PACKAGES)
  # Fetch GoogleBenchmark
  dart_fetch_git_repo(
    PROJECT_NAME benchmark
    WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/.deps
    GIT_URL https://github.com/google/benchmark.git
    GIT_TAG v1.7.1
  )

  # Set options
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)

  # Adds the targers: gtest, gtest_main, gmock, gmock_main
  add_subdirectory(
    ${benchmark_SOURCE_DIR}
    ${benchmark_BINARY_DIR}
    EXCLUDE_FROM_ALL
  )

  set(benchmark_FOUND TRUE)
else()
  find_package(benchmark)
  if(NOT benchmark_FOUND)
    message("[WARN] Skipping [benchmark] due to missing [benchmark] package.")
    return()
  endif()
endif()

set(benchmark_type "BENCHMARK")

add_subdirectory(common)

add_subdirectory(integration)

# Add custom target to build all the benchmarks
get_property(benchmark_all GLOBAL PROPERTY DART_${benchmark_type}_BENCHMARKS)
add_custom_target(benchmarks
  DEPENDS ${benchmark_all}
)
