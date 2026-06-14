# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(EnTT 3.14 QUIET CONFIG)

if(NOT EnTT_FOUND AND NOT TARGET EnTT::EnTT)
  include(FetchContent)

  FetchContent_GetProperties(entt)
  if(NOT entt_POPULATED)
    FetchContent_Populate(
      entt
      GIT_REPOSITORY https://github.com/skypjack/entt.git
      GIT_TAG v3.16.0
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
  endif()

  add_library(EnTT::EnTT INTERFACE IMPORTED GLOBAL)
  set_target_properties(
    EnTT::EnTT
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${entt_SOURCE_DIR}/src"
  )
  set(EnTT_VERSION 3.16.0 CACHE STRING "EnTT version" FORCE)
  set(EnTT_FOUND TRUE CACHE BOOL "EnTT found via FetchContent" FORCE)
endif()
