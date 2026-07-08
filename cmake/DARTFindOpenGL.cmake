# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

function(dart_prefer_prefix_glvnd_library variable)
  if(WIN32 OR APPLE OR CMAKE_CROSSCOMPILING OR DART_BUILD_WHEELS)
    return()
  endif()

  set(_dart_cached_library "${${variable}}")
  if(
    _dart_cached_library
    AND NOT _dart_cached_library MATCHES "-NOTFOUND$"
    AND NOT _dart_cached_library MATCHES "^/usr/lib(/|$)"
    AND NOT _dart_cached_library MATCHES "^/usr/lib64(/|$)"
  )
    return()
  endif()

  set(_dart_search_prefixes)
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _dart_search_prefixes "$ENV{CONDA_PREFIX}")
  endif()
  list(
    APPEND _dart_search_prefixes
    ${CMAKE_PREFIX_PATH}
    ${CMAKE_INSTALL_PREFIX}
  )
  list(REMOVE_DUPLICATES _dart_search_prefixes)

  foreach(_dart_prefix IN LISTS _dart_search_prefixes)
    if(NOT _dart_prefix)
      continue()
    endif()
    foreach(_dart_libdir lib lib64)
      foreach(_dart_library_name IN LISTS ARGN)
        set(
          _dart_candidate
          "${_dart_prefix}/${_dart_libdir}/${_dart_library_name}"
        )
        if(EXISTS "${_dart_candidate}")
          set(
            ${variable}
            "${_dart_candidate}"
            CACHE FILEPATH
            "Path to the OpenGL GLVND library"
            FORCE
          )
          return()
        endif()
      endforeach()
    endforeach()
  endforeach()
endfunction()

dart_prefer_prefix_glvnd_library(
  OPENGL_opengl_LIBRARY
  libOpenGL.so
  libOpenGL.so.0
)
dart_prefer_prefix_glvnd_library(OPENGL_glx_LIBRARY libGLX.so libGLX.so.0)

cmake_policy(PUSH)

# Use GLVND over the legacy OpenGL libraries
if(POLICY CMP0072)
  if(DART_BUILD_WHEELS)
    cmake_policy(SET CMP0072 OLD)
  else()
    cmake_policy(SET CMP0072 NEW)
  endif()
endif()

# Use OpenGL config if available
find_package(OpenGL QUIET CONFIG)
# Otherwise, fall back to FindOpenGL.cmake provided by CMake
if(NOT OPENGL_FOUND)
  find_package(OpenGL QUIET MODULE)
endif()

cmake_policy(POP)

if((OPENGL_FOUND OR OpenGL_FOUND) AND NOT TARGET OpenGL::GL)
  add_library(OpenGL::GL INTERFACE IMPORTED)
  set_target_properties(
    OpenGL::GL
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "${OPENGL_gl_LIBRARY}"
  )
endif()

if((OPENGL_FOUND OR OpenGL_FOUND) AND NOT TARGET OpenGL::GLU)
  add_library(OpenGL::GLU INTERFACE IMPORTED)
  set_target_properties(
    OpenGL::GLU
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "${OPENGL_glu_LIBRARY}"
  )
endif()
