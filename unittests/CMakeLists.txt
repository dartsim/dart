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

# Compile each test file
file(GLOB tests "*.cpp")
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

