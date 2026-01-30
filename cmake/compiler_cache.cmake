# Copyright (c) The DART development contributors
# Distributed under the BSD-style license; see LICENSE for details.

include_guard(GLOBAL)

# Configure compiler cache support shared across CI, pixi, and direct CMake builds.
# This module prefers sccache, then falls back to ccache if available. Users can
# disable auto-detection with DART_DISABLE_COMPILER_CACHE or override the
# preferred launcher via the DART_COMPILER_CACHE cache variable or environment.

dart_option(
  DART_DISABLE_COMPILER_CACHE
  "Disable automatic detection of compiler cache launchers (sccache/ccache)."
  OFF
  CATEGORY performance
)

set(
  DART_COMPILER_CACHE
  "${DART_COMPILER_CACHE}"
  CACHE STRING
  "Preferred compiler cache launcher (sccache or ccache). Leave empty to auto-detect."
)
set_property(CACHE DART_COMPILER_CACHE PROPERTY STRINGS "" sccache ccache)

function(_dart_collect_compiler_cache_candidates out_var)
  set(_candidates)

  if(DEFINED ENV{DART_COMPILER_CACHE} AND NOT "$ENV{DART_COMPILER_CACHE}" STREQUAL "")
    list(APPEND _candidates "$ENV{DART_COMPILER_CACHE}")
  endif()

  if(DART_COMPILER_CACHE)
    list(APPEND _candidates "${DART_COMPILER_CACHE}")
  endif()

  list(APPEND _candidates sccache ccache)

  set(_unique)
  foreach(_candidate IN LISTS _candidates)
    string(STRIP "${_candidate}" _candidate_trimmed)
    if(_candidate_trimmed STREQUAL "")
      continue()
    endif()
    string(TOLOWER "${_candidate_trimmed}" _candidate_lower)
    list(FIND _unique "${_candidate_lower}" _existing_index)
    if(_existing_index EQUAL -1)
      list(APPEND _unique "${_candidate_lower}")
    endif()
  endforeach()

  set(${out_var} "${_unique}" PARENT_SCOPE)
endfunction()

function(dart_configure_compiler_cache)
  if(DART_DISABLE_COMPILER_CACHE)
    message(STATUS "Compiler cache auto-detection disabled (DART_DISABLE_COMPILER_CACHE=ON)")
    return()
  endif()

  if(CMAKE_C_COMPILER_LAUNCHER OR CMAKE_CXX_COMPILER_LAUNCHER)
    message(STATUS "Compiler cache already configured (C: ${CMAKE_C_COMPILER_LAUNCHER} | CXX: ${CMAKE_CXX_COMPILER_LAUNCHER})")
    return()
  endif()

  _dart_collect_compiler_cache_candidates(_dart_cache_candidates)
  foreach(_candidate IN LISTS _dart_cache_candidates)
    unset(_cache_executable CACHE)
    unset(_cache_executable)
    find_program(_cache_executable NAMES ${_candidate})
    if(_cache_executable)
      set(CMAKE_C_COMPILER_LAUNCHER "${_cache_executable}" CACHE STRING "C compiler launcher used for caching" FORCE)
      set(CMAKE_CXX_COMPILER_LAUNCHER "${_cache_executable}" CACHE STRING "CXX compiler launcher used for caching" FORCE)
      if(CMAKE_CUDA_COMPILER)
        set(CMAKE_CUDA_COMPILER_LAUNCHER "${_cache_executable}" CACHE STRING "CUDA compiler launcher used for caching" FORCE)
      endif()

      set(DART_ACTIVE_COMPILER_CACHE "${_cache_executable}" CACHE INTERNAL "Compiler cache executable selected for this build")
      message(STATUS "Compiler cache enabled: ${_candidate} (${_cache_executable})")
      return()
    endif()
  endforeach()

  message(STATUS "Compiler cache disabled: neither sccache nor ccache found on PATH")
endfunction()
