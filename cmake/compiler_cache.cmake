# Copyright (c) The DART development contributors
# Distributed under the BSD-style license; see LICENSE for details.

include_guard(GLOBAL)

# Configure compiler cache support shared across CI, pixi, and direct CMake
# builds. Prefer sccache, then fall back to ccache if available. Users can
# disable auto-detection with DART_DISABLE_COMPILER_CACHE or force a specific
# launcher via the DART_COMPILER_CACHE cache variable or environment variable.

set(disable_compiler_cache_default OFF)
if(
  DEFINED ENV{DART_DISABLE_COMPILER_CACHE}
  AND NOT "$ENV{DART_DISABLE_COMPILER_CACHE}" STREQUAL ""
)
  set(disable_compiler_cache_default "$ENV{DART_DISABLE_COMPILER_CACHE}")
endif()

dart_option(
  DART_DISABLE_COMPILER_CACHE
  "Disable automatic detection of compiler cache launchers (sccache/ccache)."
  "${disable_compiler_cache_default}"
)
unset(disable_compiler_cache_default)

if(DART_DISABLE_COMPILER_CACHE)
  set(
    DART_DISABLE_COMPILER_CACHE
    ON
    CACHE BOOL
    "Disable automatic detection of compiler cache launchers (sccache/ccache)."
    FORCE
  )
else()
  set(
    DART_DISABLE_COMPILER_CACHE
    OFF
    CACHE BOOL
    "Disable automatic detection of compiler cache launchers (sccache/ccache)."
    FORCE
  )
endif()

set(
  DART_COMPILER_CACHE
  "${DART_COMPILER_CACHE}"
  CACHE STRING
  "Preferred compiler cache launcher (sccache or ccache). Leave empty to auto-detect."
)
set_property(CACHE DART_COMPILER_CACHE PROPERTY STRINGS "" sccache ccache)

function(_dart_collect_compiler_cache_candidates out_var)
  set(candidates)

  if(
    DEFINED ENV{DART_COMPILER_CACHE}
    AND NOT "$ENV{DART_COMPILER_CACHE}" STREQUAL ""
  )
    list(APPEND candidates "$ENV{DART_COMPILER_CACHE}")
  endif()

  if(DART_COMPILER_CACHE)
    list(APPEND candidates "${DART_COMPILER_CACHE}")
  endif()

  list(APPEND candidates sccache ccache)

  set(unique_candidates)
  set(unique_keys)
  foreach(candidate IN LISTS candidates)
    string(STRIP "${candidate}" candidate_trimmed)
    if(candidate_trimmed STREQUAL "")
      continue()
    endif()

    string(TOLOWER "${candidate_trimmed}" candidate_key)
    list(FIND unique_keys "${candidate_key}" existing_index)
    if(existing_index EQUAL -1)
      list(APPEND unique_keys "${candidate_key}")
      list(APPEND unique_candidates "${candidate_trimmed}")
    endif()
  endforeach()

  set(${out_var} "${unique_candidates}" PARENT_SCOPE)
endfunction()

function(_dart_seed_compiler_launcher_from_environment language)
  set(variable "CMAKE_${language}_COMPILER_LAUNCHER")
  if(
    NOT ${variable}
    AND DEFINED ENV{${variable}}
    AND NOT "$ENV{${variable}}" STREQUAL ""
  )
    set(
      ${variable}
      "$ENV{${variable}}"
      CACHE STRING
      "${language} compiler launcher used for caching"
      FORCE
    )
  endif()
endfunction()

function(_dart_find_compiler_cache candidate out_var)
  unset(cache_executable CACHE)
  unset(cache_executable)

  if(IS_ABSOLUTE "${candidate}" AND EXISTS "${candidate}")
    set(cache_executable "${candidate}")
  else()
    find_program(cache_executable NAMES "${candidate}")
  endif()

  set(${out_var} "${cache_executable}" PARENT_SCOPE)
endfunction()

function(dart_configure_compiler_cache)
  if(DART_DISABLE_COMPILER_CACHE)
    set(
      CMAKE_C_COMPILER_LAUNCHER
      ""
      CACHE STRING
      "C compiler launcher used for caching"
      FORCE
    )
    set(
      CMAKE_CXX_COMPILER_LAUNCHER
      ""
      CACHE STRING
      "CXX compiler launcher used for caching"
      FORCE
    )
    if(CMAKE_CUDA_COMPILER)
      set(
        CMAKE_CUDA_COMPILER_LAUNCHER
        ""
        CACHE STRING
        "CUDA compiler launcher used for caching"
        FORCE
      )
    endif()
    unset(DART_ACTIVE_COMPILER_CACHE CACHE)
    message(
      STATUS
      "Compiler cache auto-detection disabled (DART_DISABLE_COMPILER_CACHE=ON)"
    )
    return()
  endif()

  _dart_seed_compiler_launcher_from_environment(C)
  _dart_seed_compiler_launcher_from_environment(CXX)

  set(cuda_compiler_launcher_preconfigured OFF)
  if(DEFINED CMAKE_CUDA_COMPILER_LAUNCHER)
    set(cuda_compiler_launcher_preconfigured ON)
  elseif(
    DEFINED ENV{CMAKE_CUDA_COMPILER_LAUNCHER}
    AND NOT "$ENV{CMAKE_CUDA_COMPILER_LAUNCHER}" STREQUAL ""
  )
    set(cuda_compiler_launcher_preconfigured ON)
    set(
      CMAKE_CUDA_COMPILER_LAUNCHER
      "$ENV{CMAKE_CUDA_COMPILER_LAUNCHER}"
      CACHE STRING
      "CUDA compiler launcher used for caching"
      FORCE
    )
  endif()

  if(CMAKE_C_COMPILER_LAUNCHER OR CMAKE_CXX_COMPILER_LAUNCHER)
    if(CMAKE_CUDA_COMPILER AND NOT cuda_compiler_launcher_preconfigured)
      set(
        CMAKE_CUDA_COMPILER_LAUNCHER
        ""
        CACHE STRING
        "CUDA compiler launcher used for caching"
        FORCE
      )
    endif()
    message(
      STATUS
      "Compiler cache already configured (C: ${CMAKE_C_COMPILER_LAUNCHER} | CXX: ${CMAKE_CXX_COMPILER_LAUNCHER})"
    )
    return()
  endif()

  _dart_collect_compiler_cache_candidates(cache_candidates)
  foreach(candidate IN LISTS cache_candidates)
    _dart_find_compiler_cache("${candidate}" cache_executable)
    if(cache_executable)
      set(
        CMAKE_C_COMPILER_LAUNCHER
        "${cache_executable}"
        CACHE STRING
        "C compiler launcher used for caching"
        FORCE
      )
      set(
        CMAKE_CXX_COMPILER_LAUNCHER
        "${cache_executable}"
        CACHE STRING
        "CXX compiler launcher used for caching"
        FORCE
      )
      if(CMAKE_CUDA_COMPILER AND NOT cuda_compiler_launcher_preconfigured)
        set(
          CMAKE_CUDA_COMPILER_LAUNCHER
          ""
          CACHE STRING
          "CUDA compiler launcher used for caching"
          FORCE
        )
      endif()

      set(
        DART_ACTIVE_COMPILER_CACHE
        "${cache_executable}"
        CACHE INTERNAL
        "Compiler cache executable selected for this build"
      )
      message(
        STATUS
        "Compiler cache enabled: ${candidate} (${cache_executable})"
      )
      return()
    endif()
  endforeach()

  message(
    STATUS
    "Compiler cache disabled: neither sccache nor ccache found on PATH"
  )
endfunction()
