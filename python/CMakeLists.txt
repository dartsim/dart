# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

set(DART_DARTPY_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(DART_DARTPY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/dartpy")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(dartpy)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)

# Create dartpy_submodules target that contains all the active submodules
get_property(dartpy_submodules GLOBAL PROPERTY _DART_DARTPY_SUBMODULES)
add_custom_target(
  dartpy
  DEPENDS ${dartpy_submodules}
)

message(STATUS "[ dartpy ]")
message(STATUS "- PYTHON_EXECUTABLE   : ${PYTHON_EXECUTABLE}")
message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
message(STATUS "- Active submodules   : ${dartpy_submodules}")
