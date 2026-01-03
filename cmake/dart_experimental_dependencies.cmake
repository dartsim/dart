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
# Simulation-Experimental Dependency Management
#==============================================================================
# This file centralizes all dependency finding for the simulation-experimental
# module to avoid
# finding the same package multiple times and to provide clear visibility
# of all required dependencies.
#
# Dependencies are organized by category and found only once.
#==============================================================================

# Prevent multiple inclusion
if(DART_EXPERIMENTAL_DEPENDENCIES_INCLUDED)
  return()
endif()
set(DART_EXPERIMENTAL_DEPENDENCIES_INCLUDED TRUE)

# Initialize global lists for dependency tracking
set(DART_EXPERIMENTAL_DEPS_FOUND "" CACHE INTERNAL "List of found dependencies")
set(DART_EXPERIMENTAL_DEPS_MISSING "" CACHE INTERNAL "List of missing dependencies")

if(DART_EXPERIMENTAL_VERBOSE)
  message(STATUS "==================================")
  message(STATUS "  Simulation-Experimental Dependencies (Verbose)")
  message(STATUS "==================================")
endif()

#==============================================================================
# Core Dependencies (always required)
#==============================================================================
if(DART_EXPERIMENTAL_VERBOSE)
  message(STATUS "Finding core dependencies...")
endif()

# Eigen3 - Linear algebra library
dart_experimental_find_package(
  NAME Eigen3
  PACKAGE Eigen3
  VERSION 3.4
  REQUIRED
)

# EnTT - Entity Component System library
dart_experimental_find_package(
  NAME EnTT
  PACKAGE EnTT
  VERSION 3.14
  REQUIRED
)

# spdlog - Logging library
dart_experimental_find_package(
  NAME spdlog
  PACKAGE spdlog
  REQUIRED
)

# Taskflow - Parallel task programming library
dart_experimental_find_package(
  NAME Taskflow
  PACKAGE Taskflow
  REQUIRED
)

#==============================================================================
# Testing Dependencies (optional)
#==============================================================================
if(DART_EXPERIMENTAL_VERBOSE)
  message(STATUS "Finding testing dependencies...")
endif()

# GoogleTest - Unit testing framework
dart_experimental_find_package(
  NAME GTest
  PACKAGE GTest
  QUIET
  SET_VAR DART_EXPERIMENTAL_BUILD_TESTS
)

# Google Benchmark - Performance benchmarks
dart_experimental_find_package(
  NAME benchmark
  PACKAGE benchmark
  QUIET
  SET_VAR DART_EXPERIMENTAL_BUILD_BENCHMARKS
)

#==============================================================================
# Python Dependencies (optional)
#==============================================================================
if(DART_EXPERIMENTAL_VERBOSE)
  message(STATUS "Finding Python dependencies...")
endif()

# Python - Required for Python bindings
dart_experimental_find_package(
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
    dart_experimental_find_package(
      NAME nanobind
      PACKAGE nanobind
      QUIET
      SET_VAR DART_EXPERIMENTAL_BUILD_PYTHON
    )
  else()
    if(DART_EXPERIMENTAL_VERBOSE)
      message(STATUS "  ✗ nanobind not found (Python bindings disabled)")
    endif()
    set(DART_EXPERIMENTAL_BUILD_PYTHON FALSE CACHE BOOL "Build simulation-experimental Python bindings" FORCE)
  endif()
else()
  set(DART_EXPERIMENTAL_BUILD_PYTHON FALSE CACHE BOOL "Build simulation-experimental Python bindings" FORCE)
endif()

#==============================================================================
# Profiling Dependencies (optional)
#==============================================================================
if(DART_EXPERIMENTAL_VERBOSE)
  message(STATUS "Finding profiling dependencies...")
endif()

# Tracy - Frame profiler
dart_experimental_find_package(
  NAME Tracy
  PACKAGE Tracy
  QUIET
  SET_VAR DART_EXPERIMENTAL_TRACY_AVAILABLE
)

if(DART_EXPERIMENTAL_VERBOSE)
  message(STATUS "==================================")
else()
  # Compact summary - use tracked dependency lists
  list(JOIN DART_EXPERIMENTAL_DEPS_FOUND "✓ " _found_str)
  set(_dart_experimental_summary "simulation-experimental: ${_found_str}✓")

  if(DART_EXPERIMENTAL_DEPS_MISSING)
    list(JOIN DART_EXPERIMENTAL_DEPS_MISSING " " _missing_str)
    set(_dart_experimental_summary "${_dart_experimental_summary} (missing: ${_missing_str})")
  endif()

  message(STATUS "${_dart_experimental_summary}")
endif()
