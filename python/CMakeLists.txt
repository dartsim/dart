# Copyright (c) 2011-2021, The DART development contributors

# ==============================================================================
# Basic Project Setup
# ==============================================================================

set(DART_PYTHON_PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DART_PYTHON_PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(src)
add_subdirectory(test)
