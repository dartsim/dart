# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

# Check if the required libraries are available
set(DARTPY_REQUIRED_LIBS
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimization
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-gui
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
