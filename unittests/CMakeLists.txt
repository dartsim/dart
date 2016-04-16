#
# Copyright (c) 2013-2015, Georgia Tech Research Corporation
# All rights reserved.
#
# Author(s): Can Erdogan <cerdogan3@gatech.edu>,
#            Jeongseok Lee <jslee02@gmail.com>
#
# Georgia Tech Graphics Lab and Humanoid Robotics Lab
#
# Directed by Prof. C. Karen Liu and Prof. Mike Stilman
# <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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
#

# Create a macro to check if a list contains a value
macro(list_contains var value)
  set(${var})
  foreach (value2 ${ARGN})
    if(${value} STREQUAL ${value2})
      set(${var} true)
    endif (${value} STREQUAL ${value2})
  endforeach (value2)
endmacro(list_contains)

# Include and link to gtest
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/unittests/gtest/include)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/unittests/gtest)
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
if(NOT WIN32)
  target_link_libraries(gtest pthread)
endif()
set_target_properties(gtest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Compile each test file
file(GLOB tests "test*.cpp")
foreach(test ${tests})
  # Get the name (i.e. bla.cpp => bla)
  get_filename_component(base ${test} NAME_WE)
  list(APPEND all_tests ${base})

  add_executable(${base} ${test})
  dart_add_unittest(${base})

  if(MSVC)
    target_link_libraries(${base} dart dart-utils dart-utils-urdf optimized gtest debug gtestd)
  else()
    target_link_libraries(${base} dart dart-utils dart-utils-urdf gtest)
  endif()

  set_target_properties(${base} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests")

  # Add the executable if not to be ignored
  list_contains(contains ${base} ${dontTest})
  if(NOT contains)
    add_test(${base} ${CMAKE_BINARY_DIR}/bin/tests/${base})
  endif(NOT contains)
endforeach(test)


if(HAVE_IPOPT)
  target_link_libraries(testOptimizer dart-optimizer-ipopt ${IPOPT_LIBRARIES})
endif(HAVE_IPOPT)

if(HAVE_NLOPT)
  target_link_libraries(testOptimizer dart-optimizer-nlopt ${NLOPT_LIBRARIES})
endif(HAVE_NLOPT)

if(HAVE_SNOPT)
  target_link_libraries(testOptimizer dart-optimizer-snopt)
endif(HAVE_SNOPT)
