# Copyright (c) 2011-2022, The DART development contributors

# ==============================================================================
# Basic Project Setup
# ==============================================================================

set(DART_PYTHON_PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DART_PYTHON_PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Python binding module name
set(pybind_module dartpy${DART_VERSION_MAJOR})

# ==============================================================================
# Dependency Check
# ==============================================================================

if(NOT DARTPY_PYTHON_VERSION)
  set(DARTPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

if(APPLE)
  if(NOT DEFINED Python_FIND_REGISTRY)
      set(Python_FIND_REGISTRY "LAST")
  endif()
  if(NOT DEFINED Python_FIND_FRAMEWORK)
      set(Python_FIND_FRAMEWORK "LAST")
  endif()
  if(NOT DEFINED Python_FIND_STRATEGY)
      set(Python_FIND_STRATEGY "VERSION")
  endif()
endif()

find_package(Python COMPONENTS Interpreter Development)

if(NOT Python_FOUND)
  message(WARNING "Disabling [dartpy] due to missing Python.")
  return()
endif()

find_package(pybind11 CONFIG 2.2.0 QUIET)
if(NOT pybind11_FOUND)
  message(WARNING "Disabling [dartpy] due to missing pybind11 >= 2.2.0.")
  return()
endif()

# DART components
set(dart_component_targets
  ${PROJECT_NAME}${DART_VERSION_MAJOR}-common
  ${PROJECT_NAME}${DART_VERSION_MAJOR}-math
  ${PROJECT_NAME}${DART_VERSION_MAJOR}-collision
  ${PROJECT_NAME}${DART_VERSION_MAJOR}-multibody
  ${PROJECT_NAME}${DART_VERSION_MAJOR}-gui
)
foreach(target ${dart_component_targets})
  if(NOT TARGET ${target})
    message(WARNING "Skipped [dartpy${DART_VERSION_MAJOR}] due to missing component target [${target}]")
    return()
  endif()
endforeach()

# ==============================================================================
# Add subdirectories
# ==============================================================================

add_subdirectory(src)
add_subdirectory(test)
