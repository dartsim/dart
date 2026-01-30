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

if(NOT DART_BUILD_DARTPY)
  return()
endif()

set(DART_PYTHON_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}/dartpy")
set(DART_DARTPY_RUNTIME_DIR "${DART_DARTPY_BUILD_DIR}")

if(WIN32)
  set(DART_PYTHONPATH_DELIM ";")
else()
  set(DART_PYTHONPATH_DELIM ":")
endif()

set(_dart_pythonpath_components)
list(APPEND _dart_pythonpath_components "${DART_PYTHON_BUILD_DIR}")
if(DART_BUILD_DARTPY)
  list(APPEND _dart_pythonpath_components "${DART_DARTPY_RUNTIME_DIR}")
endif()

string(JOIN "${DART_PYTHONPATH_DELIM}" DART_PYTHONPATH_ENV ${_dart_pythonpath_components})
if(WIN32)
  string(REPLACE ";" "\\;" DART_PYTHONPATH_ENV "${DART_PYTHONPATH_ENV}")
endif()

if(DART_BUILD_DARTPY)
  add_subdirectory(dartpy)
  message(STATUS "")
  message(STATUS "[ dartpy ]")
  message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
  if(DEFINED PYTHON_SITE_PACKAGES)
    message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
  endif()
  if(DEFINED DARTPY_PYTEST_FOUND)
    message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
  endif()
endif()

if(DART_BUILD_DARTPY)
  add_subdirectory(tests)
  add_subdirectory(examples)
  add_subdirectory(tutorials)
endif()
