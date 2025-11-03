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

include(FetchContent)

set(BUILD_DARTPY_ENABLED ${DART_BUILD_DARTPY})
set(BUILD_DARTPY7_ENABLED ${DART_BUILD_DARTPY7})

#===============================================================================
# pybind11 (dartpy)
#===============================================================================
if(DART_BUILD_DARTPY)
  if(DART_USE_SYSTEM_PYBIND11)
    find_package(pybind11 CONFIG)
    if(NOT pybind11_FOUND)
      message(WARNING "Disabling [dartpy] due to missing pybind11 >= 2.2.0.")
      set(BUILD_DARTPY_ENABLED FALSE)
    endif()
  else()
    FetchContent_Declare(
      pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11
      GIT_TAG v3.0.1
      GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
      FetchContent_MakeAvailable(pybind11)
    endif()
    set(pybind11_FOUND TRUE)
  endif()
endif()

#===============================================================================
# nanobind (dartpy7)
#===============================================================================
set(HAVE_NANOBIND FALSE)
if(DART_BUILD_DARTPY7)
  if(NOT TARGET Python::Module)
    find_package(
      Python 3.8
      COMPONENTS Interpreter Development.Module
      OPTIONAL_COMPONENTS Development.SABIModule
    )
  endif()

  if(DART_USE_SYSTEM_NANOBIND)
    find_package(nanobind CONFIG)
    if(COMMAND nanobind_add_module)
      set(HAVE_NANOBIND TRUE)
    endif()
  else()
    FetchContent_Declare(
      nanobind
      GIT_REPOSITORY https://github.com/wjakob/nanobind.git
      GIT_TAG v2.4.0
      GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(nanobind)
    if(NOT nanobind_POPULATED)
      FetchContent_MakeAvailable(nanobind)
    endif()
    if(COMMAND nanobind_add_module)
      set(HAVE_NANOBIND TRUE)
    endif()
  endif()

  if(HAVE_NANOBIND AND TARGET nanobind)
    target_compile_options(nanobind PRIVATE -Wno-error -Wno-error=sign-compare)
  endif()

  if(NOT HAVE_NANOBIND)
    message(WARNING "Disabling [dartpy7] due to missing nanobind >= 2.4.0.")
    set(BUILD_DARTPY7_ENABLED FALSE)
  endif()
endif()

#===============================================================================
# dartpy (pybind11)
#===============================================================================
if(BUILD_DARTPY_ENABLED)
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

#===============================================================================
# dartpy7 (nanobind)
#===============================================================================
if(BUILD_DARTPY7_ENABLED)
  add_subdirectory(dartpy7)

  message(STATUS "")
  message(STATUS "[ dartpy7 ]")
  message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
  message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
endif()
