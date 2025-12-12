# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

cmake_policy(PUSH)

# Prefer a config package if available (e.g., from conda-forge). Fall back to
# CMake's FindGLUT module otherwise.
find_package(GLUT QUIET CONFIG)
if(NOT GLUT_FOUND)
  find_package(GLUT QUIET MODULE)
endif()

cmake_policy(POP)

# Some environments (e.g., pixi/conda on ubuntu-latest) ship freeglut without
# system X11 development packages, which can cause CMake's FindGLUT module to
# fail even though a usable libglut is present under the prefix. As a fallback,
# locate freeglut directly from CMAKE_PREFIX_PATH/CONDA_PREFIX.
if(NOT GLUT_FOUND)
  find_path(GLUT_INCLUDE_DIR
    NAMES GL/glut.h
    PATH_SUFFIXES include
  )

  find_library(GLUT_glut_LIBRARY
    NAMES glut freeglut
    PATH_SUFFIXES lib
  )

  if(GLUT_INCLUDE_DIR AND GLUT_glut_LIBRARY)
    set(GLUT_FOUND TRUE)
  endif()
endif()

if(GLUT_FOUND AND NOT TARGET GLUT::GLUT)
  add_library(GLUT::GLUT INTERFACE IMPORTED)
  set_target_properties(GLUT::GLUT PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GLUT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${GLUT_glut_LIBRARY}"
  )
endif()
