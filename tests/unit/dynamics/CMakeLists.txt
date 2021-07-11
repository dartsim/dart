# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(NOT TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics)
  return()
endif()

dart_format_add(
  GeneratedWamIkFast.cpp
  SharedLibraryWamIkFast.hpp
  SharedLibraryWamIkFast.cpp
)

# Glob all the test files
file(GLOB_RECURSE test_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test_*.cpp")
if(test_files)
  list(SORT test_files)
endif()

if(BUILD_SHARED_LIBS)
  dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics)
  target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(SharedLibraryWamIkFast PRIVATE -w)
  
  dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics)
  target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(GeneratedWamIkFast PRIVATE -w)

  list(APPEND link_libraries SharedLibraryWamIkFast GeneratedWamIkFast)
else()
  list(REMOVE_ITEM test_files test_IkFast.cpp)
endif()

set(link_dart_libraries
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
)

# Build the uint tests
dart_build_tests(
  TYPE ${test_type}
  TEST_LIST test_targets
  SOURCES ${test_files}
  LINK_LIBRARIES ${link_libraries}
  LINK_DART_LIBRARIES ${link_dart_libraries}
  TARGET_PREFIX dynamics
)
