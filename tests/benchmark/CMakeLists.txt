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

set(benchmark_type "BENCHMARK_INTEGRATION")

# Glob all the benchmark files
file(GLOB_RECURSE benchmark_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "benchmark_*.cpp")
if(benchmark_files)
  list(SORT benchmark_files)
endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  set(link_libraries)
  set(link_dart_libraries
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
  )

  dart_build_benchmarks(
    TYPE ${benchmark_type}
    TEST_LIST integration_benchmarks
    SOURCES
      benchmark_Kinematics.cpp
    LINK_LIBRARIES ${link_libraries}
    LINK_DART_LIBRARIES ${link_dart_libraries}
  )
endif()
