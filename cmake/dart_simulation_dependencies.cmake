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

#==============================================================================
# Simulation Dependency Management
#==============================================================================
# This file centralizes all dependency finding for the simulation
# module to avoid
# finding the same package multiple times and to provide clear visibility
# of all required dependencies.
#
# Dependencies are organized by category and found only once.
#==============================================================================

function(dart_simulation_find_package)
  dart_find_dependency(PREFIX DART_SIMULATION ${ARGN})
endfunction()

function(dart_simulation_enable_benchmark_target)
  if(NOT DART_SIMULATION_BUILD_BENCHMARKS AND TARGET benchmark::benchmark)
    if(DART_SIMULATION_DEPS_MISSING)
      list(REMOVE_ITEM DART_SIMULATION_DEPS_MISSING benchmark)
      set(
        DART_SIMULATION_DEPS_MISSING
        "${DART_SIMULATION_DEPS_MISSING}"
        CACHE INTERNAL
        "List of missing dependencies"
      )
    endif()
    list(APPEND DART_SIMULATION_DEPS_FOUND benchmark)
    set(
      DART_SIMULATION_DEPS_FOUND
      "${DART_SIMULATION_DEPS_FOUND}"
      CACHE INTERNAL
      "List of found dependencies"
    )
    set(
      DART_SIMULATION_BUILD_BENCHMARKS
      TRUE
      CACHE BOOL
      "benchmark available"
      FORCE
    )
    if(DART_SIMULATION_VERBOSE)
      message(STATUS "  ✓ benchmark (target already available)")
    endif()
  endif()
endfunction()

# Prevent multiple inclusion
if(DART_SIMULATION_DEPENDENCIES_INCLUDED)
  dart_simulation_enable_benchmark_target()
  return()
endif()
set(DART_SIMULATION_DEPENDENCIES_INCLUDED TRUE)

# Initialize global lists for dependency tracking
set(DART_SIMULATION_DEPS_FOUND "" CACHE INTERNAL "List of found dependencies")
set(
  DART_SIMULATION_DEPS_MISSING
  ""
  CACHE INTERNAL
  "List of missing dependencies"
)

if(DART_SIMULATION_VERBOSE)
  message(STATUS "==================================")
  message(STATUS "  Simulation Dependencies (Verbose)")
  message(STATUS "==================================")
endif()

#==============================================================================
# Core Dependencies (always required)
#==============================================================================
if(DART_SIMULATION_VERBOSE)
  message(STATUS "Finding core dependencies...")
endif()

# Eigen3 - Linear algebra library
# Reuse core DART's already-resolved Eigen3::Eigen target (core finds it as a
# hard requirement before this module is added) instead of a brittle REQUIRED
# re-find. Mirrors the EnTT/Taskflow/spdlog pattern below.
if(NOT TARGET Eigen3::Eigen)
  dart_find_package(Eigen3)
endif()
if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "Eigen3 is required for simulation")
endif()
if(DEFINED Eigen3_VERSION AND Eigen3_VERSION VERSION_LESS 5.0)
  message(
    FATAL_ERROR
    "Eigen version>=5.0 is required, but found ${Eigen3_VERSION}"
  )
endif()
# Keep Eigen3 in the tracked-deps summary so the status line still prints it.
list(APPEND DART_SIMULATION_DEPS_FOUND Eigen3)
set(
  DART_SIMULATION_DEPS_FOUND
  "${DART_SIMULATION_DEPS_FOUND}"
  CACHE INTERNAL
  "List of found dependencies"
)

# EnTT - Entity Component System library
# Use the fallback-capable finder so clean environments (containers, wheel
# builds) auto-fetch EnTT instead of FATAL_ERRORing on a plain
# find_package(... REQUIRED). Mirrors dart/collision/native/CMakeLists.txt.
if(NOT TARGET EnTT::EnTT)
  dart_find_package(EnTT)
endif()
if(NOT TARGET EnTT::EnTT)
  message(FATAL_ERROR "EnTT >= 3.14 is required for simulation")
endif()
# Keep EnTT in the tracked-deps summary so the status line still prints it.
list(APPEND DART_SIMULATION_DEPS_FOUND EnTT)
set(
  DART_SIMULATION_DEPS_FOUND
  "${DART_SIMULATION_DEPS_FOUND}"
  CACHE INTERNAL
  "List of found dependencies"
)

# spdlog - Logging library
# spdlog is optional for core DART (core links it only if found and otherwise
# builds with DART_HAVE_spdlog=0), but the simulation module hard-requires the
# spdlog::spdlog target (logging.hpp includes spdlog unconditionally and the
# library links it PRIVATE). A plain
# find_package(... REQUIRED) HARD-FAILS in clean environments lacking
# spdlogConfig.cmake (e.g. the Alt Linux Docker lane). Route through the
# fallback-capable finder, which reuses core's target when present and otherwise
# FetchContent-fetches spdlog. Mirrors EnTT/Taskflow.
if(NOT TARGET spdlog::spdlog)
  dart_find_package(spdlog)
endif()
if(NOT TARGET spdlog::spdlog)
  message(FATAL_ERROR "spdlog is required for simulation")
endif()
# Keep spdlog in the tracked-deps summary so the status line still prints it.
list(APPEND DART_SIMULATION_DEPS_FOUND spdlog)
set(
  DART_SIMULATION_DEPS_FOUND
  "${DART_SIMULATION_DEPS_FOUND}"
  CACHE INTERNAL
  "List of found dependencies"
)

# Taskflow - Parallel task programming library
# Use the fallback-capable finder so clean environments auto-fetch Taskflow
# instead of FATAL_ERRORing on a plain find_package(... REQUIRED).
if(NOT TARGET Taskflow::Taskflow)
  dart_find_package(Taskflow)
endif()
if(NOT TARGET Taskflow::Taskflow)
  message(FATAL_ERROR "Taskflow is required for simulation")
endif()
# Keep Taskflow in the tracked-deps summary so the status line still prints it.
list(APPEND DART_SIMULATION_DEPS_FOUND Taskflow)
set(
  DART_SIMULATION_DEPS_FOUND
  "${DART_SIMULATION_DEPS_FOUND}"
  CACHE INTERNAL
  "List of found dependencies"
)

#==============================================================================
# Testing Dependencies (optional)
#==============================================================================
if(DART_SIMULATION_VERBOSE)
  message(STATUS "Finding testing dependencies...")
endif()

# GoogleTest - Unit testing framework
dart_simulation_find_package(
  NAME GTest
  PACKAGE GTest
  QUIET
  SET_VAR DART_SIMULATION_BUILD_TESTS
)

# Google Benchmark - Performance benchmarks
dart_simulation_find_package(
  NAME benchmark
  PACKAGE benchmark
  QUIET
  SET_VAR DART_SIMULATION_BUILD_BENCHMARKS
)
dart_simulation_enable_benchmark_target()

#==============================================================================
# Python Dependencies (optional)
#==============================================================================
if(DART_SIMULATION_VERBOSE)
  message(STATUS "Finding Python dependencies...")
endif()

# Python - Required for Python bindings
dart_simulation_find_package(
  NAME Python
  PACKAGE Python
  COMPONENTS Interpreter Development
  QUIET
)

# nanobind - Python binding library (requires Python)
if(Python_FOUND)
  # Find nanobind using Python
  execute_process(
    COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
    OUTPUT_VARIABLE nanobind_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE nanobind_NOTFOUND
  )

  if(NOT nanobind_NOTFOUND)
    list(APPEND CMAKE_PREFIX_PATH "${nanobind_ROOT}")
    dart_simulation_find_package(
      NAME nanobind
      PACKAGE nanobind
      QUIET
      SET_VAR DART_SIMULATION_BUILD_PYTHON
    )
  else()
    if(DART_SIMULATION_VERBOSE)
      message(STATUS "  ✗ nanobind not found (Python bindings disabled)")
    endif()
    set(
      DART_SIMULATION_BUILD_PYTHON
      FALSE
      CACHE BOOL
      "Build simulation Python bindings"
      FORCE
    )
  endif()
else()
  set(
    DART_SIMULATION_BUILD_PYTHON
    FALSE
    CACHE BOOL
    "Build simulation Python bindings"
    FORCE
  )
endif()

#==============================================================================
# Profiling Dependencies (optional)
#==============================================================================
if(DART_SIMULATION_VERBOSE)
  message(STATUS "Finding profiling dependencies...")
endif()

# Tracy - Frame profiler
dart_simulation_find_package(
  NAME Tracy
  PACKAGE Tracy
  QUIET
  SET_VAR DART_SIMULATION_TRACY_AVAILABLE
)

if(DART_SIMULATION_VERBOSE)
  message(STATUS "==================================")
else()
  # Compact summary - use tracked dependency lists
  list(JOIN DART_SIMULATION_DEPS_FOUND "✓ " _found_str)
  set(_dart_simulation_summary "simulation: ${_found_str}✓")

  if(DART_SIMULATION_DEPS_MISSING)
    list(JOIN DART_SIMULATION_DEPS_MISSING " " _missing_str)
    set(
      _dart_simulation_summary
      "${_dart_simulation_summary} (missing: ${_missing_str})"
    )
  endif()

  message(STATUS "${_dart_simulation_summary}")
endif()
