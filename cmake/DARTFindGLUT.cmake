# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

if(WIN32 AND NOT CYGWIN)
  set(GLUT_FOUND TRUE)
  set(GLUT_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
  set(GLUT_LIBRARIES glut32)
  set(HAVE_GLUT TRUE)
else()
  find_package(GLUT QUIET MODULE)
  if(GLUT_FOUND)
    set(HAVE_GLUT TRUE)
  else()
    set(HAVE_GLUT FALSE)
  endif()
endif()
