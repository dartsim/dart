#
#  Copyright (c) 2013-2015, Georgia Tech Research Corporation
#  All rights reserved.
#
#  Author(s): Can Erdogan <cerdogan3@gatech.edu>,
#             Jeongseok Lee <jslee02@gmail.com>
#
#  Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
#
#  Directed by Prof. C. Karen Liu and Prof. Mike Stilman
#  <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
#
#  This file is provided under the following "BSD-style" License:
#    Redistribution and use in source and binary forms, with or
#    without modification, are permitted provided that the following
#    conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#    USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#    POSSIBILITY OF SUCH DAMAGE.
#

# @file CMakeLists.txt
# @author Can Erdogan
# @date Feb 02, 2013
# @brief The CMakefile for the tests for the RRT implementations. NOTE Assumes every .cpp file is
# a test file with a "main" function.

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
if(NOT WIN32)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(GLOB gtest_hdrs "/usr/include/gtest/*.h" "/usr/include/gtest/internal/*.h")
    include_directories(/usr/include/gtest)
    include_directories(/usr/src/gtest)
    add_library(gtest /usr/src/gtest/src/gtest-all.cc ${gtest_hdrs})
  elseif(APPLE)
    add_definitions (-DGTEST_USE_OWN_TR1_TUPLE)
    file(GLOB gtest_hdrs "/usr/local/share/gtest/include/*.h" "/usr/local/share/gtest/include/internal/*.h")
    include_directories(/usr/local/share/gtest/include)
    include_directories(/usr/local/share/gtest)
    add_library(gtest /usr/local/share/gtest/src/gtest-all.cc ${gtest_hdrs})
  endif()

  target_link_libraries(gtest pthread)
  set_target_properties (gtest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

# VC11 contains std::tuple with variadic templates emulation macro.
# _VARIADIC_MAX defaulted to 5 but gtest requires 10.
if (MSVC_VERSION EQUAL 1700)
  add_definitions(/D_VARIADIC_MAX=10)
endif()

# Compile each test file
file(GLOB tests "test*.cpp")
foreach(test ${tests})

	# Get the name (i.e. bla.cpp => bla)
	get_filename_component(base ${test} NAME_WE)
	link_directories(${DARTExt_LIBRARY_DIRS})
	add_executable(${base} ${test})
	if(MSVC)
		target_link_libraries(${base} dart optimized gtest debug gtestd)
	else()
		target_link_libraries(${base} dart gtest)
	endif()

	set_target_properties(${base} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests")

	# Add the executable if not to be ignored
	list_contains(contains ${base} ${dontTest})
	if(NOT contains)
		add_test(${base} ${CMAKE_BINARY_DIR}/bin/tests/${base})
		message("-- Adding test: " ${base})
	endif(NOT contains)
endforeach(test)

