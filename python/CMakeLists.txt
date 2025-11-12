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

if(NOT DART_BUILD_DARTPY AND NOT DART_BUILD_DARTPY8)
  return()
endif()

# Set up pybind11
if(DART_USE_SYSTEM_PYBIND11)
  if(Python3_EXECUTABLE)
    execute_process(
      COMMAND
        ${Python3_EXECUTABLE}
        "-c"
        "import sysconfig, pathlib; path = pathlib.Path(sysconfig.get_path('platlib')) / 'pybind11' / 'share' / 'cmake' / 'pybind11'; print(path if path.exists() else '')"
      OUTPUT_VARIABLE PYBIND11_CONFIG_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()
  if(PYBIND11_CONFIG_DIR AND EXISTS "${PYBIND11_CONFIG_DIR}")
    set(pybind11_DIR "${PYBIND11_CONFIG_DIR}" CACHE PATH "Path to pybind11 CMake package discovered via Python" FORCE)
  endif()
  find_package(pybind11 CONFIG REQUIRED)
else()
  include(FetchContent)
  # Fetch latest stable pybind11
  # Note: Update this periodically to match the version in pixi.toml
  # Latest stable as of January 2025: v3.0.1
  # Check https://github.com/pybind/pybind11/releases for newer versions
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v3.0.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(NOT pybind11_FOUND)
  message(WARNING "Disabling [dartpy] due to missing pybind11 >= 2.2.0.")
  return()
endif()

set(DART_PYBIND11_EXTRA_DEFINITIONS PYBIND11_DISABLE_NATIVE_ENUM)

set(DART_PYTHON_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}/dartpy")

get_property(DART_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(DART_IS_MULTI_CONFIG)
  set(DART_DARTPY_RUNTIME_DIR "${DART_DARTPY_BUILD_DIR}/$<CONFIG>")
else()
  set(DART_DARTPY_RUNTIME_DIR "${DART_DARTPY_BUILD_DIR}")
endif()

if(WIN32)
  set(DART_PYTHONPATH_ENV "${DART_DARTPY_RUNTIME_DIR};${DART_PYTHON_BUILD_DIR}")
else()
  set(DART_PYTHONPATH_ENV "${DART_DARTPY_RUNTIME_DIR}:${DART_PYTHON_BUILD_DIR}")
endif()

if(DART_BUILD_DARTPY)
  add_subdirectory(dartpy)
  add_subdirectory(tests)
  add_subdirectory(examples)
  add_subdirectory(tutorials)

  message(STATUS "")
  message(STATUS "[ dartpy ]")
  message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
  message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
  message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
endif()

if(DART_BUILD_DARTPY8)
  add_subdirectory(dartpy8)

  message(STATUS "")
  message(STATUS "[ dartpy8 ]")
  message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
  message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
endif()
