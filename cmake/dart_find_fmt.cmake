# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(DART_USE_SYSTEM_FMT)
  find_package(fmt)
else()
  # System fmt is unavailable or its packaged CMake config is broken (e.g. the
  # Alt Linux Docker repro, whose rolling libfmt-devel has shipped faulty
  # fmt-targets exports). Build fmt from source instead, mirroring
  # dart_find_spdlog.cmake / dart_find_entt.cmake. Pin the same version DART
  # builds against elsewhere (pixi: fmt >=12.1.0,<13).
  if(NOT TARGET fmt::fmt)
    include(FetchContent)

    FetchContent_Declare(
      fmt
      GIT_REPOSITORY https://github.com/fmtlib/fmt.git
      GIT_TAG 12.1.0
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
  endif()

  set(fmt_VERSION 12.1.0 CACHE STRING "fmt version" FORCE)
  set(fmt_FOUND TRUE CACHE BOOL "fmt found via FetchContent" FORCE)
endif()
