# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

# Check if the required libraries are available
set(DARTPY_REQUIRED_LIBS
  dart
  dart-io
  dart-io-urdf
  dart-gui
  dart-gui-osg
)
foreach(lib ${DARTPY_REQUIRED_LIBS})
  if(NOT TARGET ${lib})
    message(WARNING "Skipped dartpy due to missing ${lib}!")
    return()
  endif()
endforeach()

set(DART_DARTPY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/dartpy")

add_subdirectory(dartpy)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)

message(STATUS "")
message(STATUS "[ dartpy ]")
message(STATUS "- PYTHON_EXECUTABLE   : ${PYTHON_EXECUTABLE}")
message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
