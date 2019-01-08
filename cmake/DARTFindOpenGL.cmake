# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(OpenGL QUIET)

if(OPENGL_FOUND AND NOT TARGET OpenGL::GL)
  add_library(OpenGL::GL INTERFACE IMPORTED)
  set_target_properties(OpenGL::GL PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${OPENGL_gl_LIBRARY}"
  )
endif()

if(OPENGL_FOUND AND NOT TARGET OpenGL::GLU)
  add_library(OpenGL::GLU INTERFACE IMPORTED)
  set_target_properties(OpenGL::GLU PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${OPENGL_glu_LIBRARY}"
  )
endif()
