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

#===============================================================================
# Usage:
#   dartpy_add_example(example_name) # assumed source is example_name.py
#   dartpy_add_example(example_name main.py)
#===============================================================================
function(dartpy_add_example example_name) # ARGN for source file
  if(${ARGC} GREATER 1)
    set(source ${ARGN})
  else()
    set(source "${example_name}.py")
  endif()
  add_custom_target(
    ${example_name}
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_PYTHONPATH_ENV} ${Python3_EXECUTABLE} ${source}"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${DART_PYTHONPATH_ENV}" ${Python3_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
  )
endfunction()

# Automatically identify main.py entries in top-level example directories.
file(GLOB example_sources RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*/main.py")
list(SORT example_sources)
foreach(example_source ${example_sources})
  get_filename_component(example_dir "${example_source}" DIRECTORY)
  get_filename_component(example_name "${example_dir}" NAME)
  if(DART_VERBOSE)
    message(STATUS "Adding Python example: " py_example_${example_name})
  endif()
  dartpy_add_example(
    py_example_${example_name}
    "${CMAKE_CURRENT_LIST_DIR}/${example_source}"
  )
endforeach()
