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

if(NOT DART_BUILD_DARTPY AND NOT DART_BUILD_DARTPY7)
  return()
endif()

# Set up pybind11
if(DART_USE_SYSTEM_PYBIND11)
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

# Patch native_enum_data.h for pybind11 3.0.1 (see https://github.com/pybind/pybind11/pull/4854)
set(DART_PYBIND11_NATIVE_ENUM_PATCH
    "${CMAKE_SOURCE_DIR}/pybind11/detail/native_enum_data.h")
if(EXISTS "${DART_PYBIND11_NATIVE_ENUM_PATCH}")
  if(NOT pybind11_INCLUDE_DIRS)
    get_target_property(pybind11_INCLUDE_DIRS pybind11::module
                        INTERFACE_INCLUDE_DIRECTORIES)
  endif()
  foreach(pybind11_inc_dir IN LISTS pybind11_INCLUDE_DIRS)
    if(EXISTS "${pybind11_inc_dir}/pybind11/detail/native_enum_data.h")
      configure_file(
        "${DART_PYBIND11_NATIVE_ENUM_PATCH}"
        "${pybind11_inc_dir}/pybind11/detail/native_enum_data.h"
        COPYONLY)
    endif()
  endforeach()
endif()

if(DART_BUILD_DARTPY)
  set(DART_DARTPY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/dartpy")

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

if(DART_BUILD_DARTPY7)
  add_subdirectory(dartpy7)

  message(STATUS "")
  message(STATUS "[ dartpy7 ]")
  message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
  message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
endif()
