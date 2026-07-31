# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(EnTT 4.0 QUIET CONFIG)

if(NOT EnTT_FOUND AND NOT TARGET EnTT::EnTT)
  include(FetchContent)

  FetchContent_GetProperties(entt)
  if(NOT entt_POPULATED)
    FetchContent_Populate(
      entt
      GIT_REPOSITORY https://github.com/skypjack/entt.git
      GIT_TAG v4.0.0
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
  endif()

  # Hand-roll the imported target instead of adding EnTT's own CMake project.
  # Mirror the compile feature upstream's EnTT target declares: EnTT 4 requires
  # C++20, so consumers reached through this fallback get the same language
  # baseline they would from EnTTConfig.cmake.
  add_library(EnTT::EnTT INTERFACE IMPORTED GLOBAL)
  set_target_properties(
    EnTT::EnTT
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${entt_SOURCE_DIR}/src"
      INTERFACE_COMPILE_FEATURES cxx_std_20
  )
  set(EnTT_VERSION 4.0.0 CACHE STRING "EnTT version" FORCE)
  set(EnTT_FOUND TRUE CACHE BOOL "EnTT found via FetchContent" FORCE)
endif()
