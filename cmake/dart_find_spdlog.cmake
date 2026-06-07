# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(spdlog 1.9.2 QUIET CONFIG)

# spdlog is OPTIONAL for core DART (it builds with DART_HAVE_spdlog=0 when absent)
# but HARD-REQUIRED by the simulation module. Clean environments
# (containers, wheel/source builds such as the Alt Linux Docker lane) may lack a
# packaged spdlogConfig.cmake, so provide a FetchContent fallback that builds the
# spdlog::spdlog target consumers link against -- mirroring dart_find_entt.cmake /
# dart_find_taskflow.cmake.
#
# This fallback is intentionally UNGATED: the installed simulation
# component config re-includes this finder to recreate spdlog::spdlog for
# downstream static consumers. Core's OPTIONAL spdlog probe avoids an unwanted
# fetch by calling find_package directly (not this finder), so core-only and
# offline configures keep the quiet optional behavior. System packages are still
# preferred; this only triggers when find_package fails.
if(NOT spdlog_FOUND AND NOT TARGET spdlog::spdlog)
  include(FetchContent)

  FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.15.1
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )

  # Disable upstream extras; spdlog bundles its own fmt so it is self-contained.
  set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_EXAMPLE_HO OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_TESTS_HO OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
  set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
  set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)

  FetchContent_MakeAvailable(spdlog)

  set(spdlog_VERSION 1.15.1 CACHE STRING "spdlog version" FORCE)
  set(spdlog_FOUND TRUE CACHE BOOL "spdlog found via FetchContent" FORCE)
endif()
