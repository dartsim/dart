# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

set(component_name io)

if(NOT TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-${component_name})
  return()
endif()

# Glob all the test files
file(GLOB_RECURSE test_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test_*.cpp")
if(test_files)
  list(SORT test_files)
endif()

set(link_libraries)
set(link_dart_libraries
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-${component_name}
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
)

# Build the uint tests
dart_build_tests(
  TYPE ${test_type}
  TEST_LIST test_targets
  SOURCES ${test_files}
  LINK_LIBRARIES ${link_libraries}
  LINK_DART_LIBRARIES ${link_dart_libraries}
  TARGET_PREFIX ${component_name}
)
