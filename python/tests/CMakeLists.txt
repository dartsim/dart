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

# Make sure pytest is installed
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import pytest; print(pytest.__version__)"
  RESULT_VARIABLE pytest_not_found
  OUTPUT_VARIABLE pytest_version
  ERROR_QUIET
)
if(pytest_not_found)
  message(WARNING "Running the tests requires pytest. Please install it manually"
    " (try: ${Python3_EXECUTABLE} -m pip install pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
elseif(pytest_version VERSION_LESS 3.0)
  message(WARNING "Running the tests requires pytest >= 3.0. Found: ${pytest_version}"
    "Please update it (try: ${Python3_EXECUTABLE} -m pip install -U pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
else()
  set(DARTPY_PYTEST_FOUND TRUE)
endif()
set(DARTPY_PYTEST_FOUND ${DARTPY_PYTEST_FOUND} PARENT_SCOPE)

set(dartpy_test_utils
  util.py
)

set(dartpy_pybind_tests_dir ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE dartpy_pybind_tests "${dartpy_pybind_tests_dir}/test_*.py")

# When dartpy_nb is enabled we re-use the canonical unit tests to exercise the
# nanobind module instead of dartpy (pybind11). Skip scheduling them for the
# pybind runner so we don't execute mismatched imports.
if(DART_BUILD_DARTPY_NB)
  set(dartpy_pybind_tests "")
endif()

set(dartpy_nb_tests_dirs
  "${CMAKE_CURRENT_SOURCE_DIR}/../test_nb"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
set(dartpy_nanobind_tests "")
foreach(_nb_dir IN LISTS dartpy_nb_tests_dirs)
  if(EXISTS "${_nb_dir}")
    file(GLOB_RECURSE _nb_tests "${_nb_dir}/test_*.py")
    list(APPEND dartpy_nanobind_tests ${_nb_tests})
  endif()
endforeach()
list(REMOVE_DUPLICATES dartpy_nanobind_tests)

set(_pytest_dependencies)
if(DARTPY_PYTEST_FOUND AND dartpy_pybind_tests AND TARGET dartpy)
  add_custom_target(pytest_pybind
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest [pybind11] with: PYTHONPATH=${DART_PYTHONPATH_ENV} ${Python3_EXECUTABLE} -m pytest [dartpy]"
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH="${DART_PYTHONPATH_ENV}" "${Python3_EXECUTABLE}" -m pytest ${dartpy_pybind_tests} -v
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${dartpy_pybind_tests} ${dartpy_test_utils}
    DEPENDS dartpy
  )
  list(APPEND _pytest_dependencies pytest_pybind)
elseif(dartpy_pybind_tests AND NOT TARGET dartpy)
  add_custom_target(pytest_pybind
    COMMAND ${CMAKE_COMMAND} -E echo "Skipping pybind11 tests because dartpy target is not built."
  )
  list(APPEND _pytest_dependencies pytest_pybind)
endif()

if(DARTPY_PYTEST_FOUND AND dartpy_nanobind_tests AND TARGET dartpy_nb)
  add_custom_target(pytest_nb
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest [nanobind] with: PYTHONPATH=${DART_PYTHONPATH_ENV} ${Python3_EXECUTABLE} -m pytest [dartpy_nb]"
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH="${DART_PYTHONPATH_ENV}" "${Python3_EXECUTABLE}" -m pytest ${dartpy_nanobind_tests} -v
    WORKING_DIRECTORY ${dartpy_nb_tests_dir}
    SOURCES ${dartpy_nanobind_tests} ${dartpy_test_utils}
    DEPENDS dartpy_nb
  )
  list(APPEND _pytest_dependencies pytest_nb)
elseif(dartpy_nanobind_tests AND NOT TARGET dartpy_nb)
  add_custom_target(pytest_nb
    COMMAND ${CMAKE_COMMAND} -E echo "Skipping nanobind tests because dartpy_nb target is not built."
  )
  list(APPEND _pytest_dependencies pytest_nb)
endif()

if(_pytest_dependencies)
  add_custom_target(pytest DEPENDS ${_pytest_dependencies})
else()
  add_custom_target(pytest
    COMMAND ${CMAKE_COMMAND} -E echo "Warning: No Python tests are configured for the current build."
  )
endif()

#===============================================================================
# Usage:
#   dartpy_add_test(test_name dep source.py [WORKING_DIRECTORY dir])
#===============================================================================
function(dartpy_add_test test_name dependency_target source)
  if(NOT TARGET ${dependency_target})
    return()
  endif()
  set(options)
  set(oneValueArgs WORKING_DIRECTORY)
  cmake_parse_arguments(DARTPY_ADD_TEST "${options}" "${oneValueArgs}" "" ${ARGN})
  set(_workdir ${DARTPY_ADD_TEST_WORKING_DIRECTORY})
  if(NOT _workdir)
    set(_workdir ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
  add_custom_target(${test_name}
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_PYTHONPATH_ENV} ${Python3_EXECUTABLE} ${source}"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${DART_PYTHONPATH_ENV}" ${Python3_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${_workdir}
    SOURCES ${source}
    DEPENDS ${dependency_target}
  )
endfunction()

if(TARGET dartpy)
  foreach(dartpy_test_file ${dartpy_pybind_tests})
    get_filename_component(dartpy_test_name ${dartpy_test_file} NAME_WE)
    dartpy_add_test(py_${dartpy_test_name} dartpy ${dartpy_test_file})
  endforeach()
endif()

if(TARGET dartpy_nb)
  foreach(dartpy_test_file ${dartpy_nanobind_tests})
    file(RELATIVE_PATH _dartpy_rel_path "${CMAKE_SOURCE_DIR}/python" "${dartpy_test_file}")
    string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _dartpy_rel_path "${_dartpy_rel_path}")
    set(_dartpy_target_name "py_nb_${_dartpy_rel_path}")
    get_filename_component(_dartpy_workdir "${dartpy_test_file}" DIRECTORY)
    dartpy_add_test(${_dartpy_target_name} dartpy_nb ${dartpy_test_file}
      WORKING_DIRECTORY ${_dartpy_workdir})
  endforeach()
endif()
