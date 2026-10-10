# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Downstream `find_package(DART)` consumers re-include this finder through
# the installed DART config but never define DART_USE_SYSTEM_FMT (it is a DART
# build-time option, not exported into the installed config). Treat an undefined
# value as ON so installed/packaged and offline consumers keep resolving the
# system fmt via find_package(fmt) instead of fetching it over the network.
if(NOT DEFINED DART_USE_SYSTEM_FMT)
  set(DART_USE_SYSTEM_FMT ON)
endif()

if(DART_USE_SYSTEM_FMT)
  find_package(fmt 8.1.1)
else()
  include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")
  # System fmt is unavailable or its packaged CMake config is broken (e.g. the
  # Alt Linux Docker repro, whose rolling libfmt-devel has shipped faulty
  # fmt-targets exports). Build fmt from source instead. Pin the same version
  # DART builds against elsewhere (pixi: fmt >=11.1.4,<12).
  if(NOT TARGET fmt::fmt)
    include(FetchContent)

    FetchContent_Declare(
      fmt
      GIT_REPOSITORY https://github.com/fmtlib/fmt.git
      GIT_TAG 11.1.4
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )

    # Keep FMT_INSTALL ON so fmt's targets belong to an export set: DART links
    # fmt::fmt-header-only PUBLIC and validates install(EXPORT) at generate time,
    # which fails if the fetched fmt targets are in no export set. Skip the rest.
    set(FMT_INSTALL ON CACHE BOOL "" FORCE)
    set(FMT_TEST OFF CACHE BOOL "" FORCE)
    set(FMT_DOC OFF CACHE BOOL "" FORCE)
    set(FMT_FUZZ OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(fmt)
    set(fmt_VERSION 11.1.4 CACHE STRING "fmt version" FORCE)
  elseif(NOT fmt_VERSION)
    get_target_property(
      _fmt_include_dirs
      fmt::fmt
      INTERFACE_INCLUDE_DIRECTORIES
    )
    dart_read_header_version(_fmt_header_version fmt/base.h FMT_VERSION ${_fmt_include_dirs})
    if(NOT _fmt_header_version)
      dart_read_header_version(_fmt_header_version fmt/core.h FMT_VERSION ${_fmt_include_dirs})
    endif()
    if(_fmt_header_version MATCHES "^[0-9]+$")
      math(EXPR _fmt_major "${_fmt_header_version} / 10000")
      math(EXPR _fmt_minor "${_fmt_header_version} / 100 % 100")
      math(EXPR _fmt_patch "${_fmt_header_version} % 100")
      set(fmt_VERSION "${_fmt_major}.${_fmt_minor}.${_fmt_patch}")
    endif()
  endif()

  set(fmt_FOUND TRUE CACHE BOOL "fmt found via FetchContent" FORCE)
  dart_check_package_version(fmt 8.1.1)
  if(NOT fmt_FOUND)
    message(FATAL_ERROR "fmt >= 8.1.1 with version information is required")
  endif()
endif()
