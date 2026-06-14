# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(Taskflow QUIET CONFIG)

if(NOT Taskflow_FOUND AND NOT TARGET Taskflow::Taskflow)
  include(FetchContent)

  FetchContent_GetProperties(taskflow)
  if(NOT taskflow_POPULATED)
    FetchContent_Populate(
      taskflow
      GIT_REPOSITORY https://github.com/taskflow/taskflow.git
      GIT_TAG v4.0.0
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
  endif()

  # Taskflow is header-only. Avoid add_subdirectory(), which would configure the
  # upstream examples/tests; expose the headers via an imported interface target
  # matching the Taskflow::Taskflow name consumers link against.
  add_library(Taskflow::Taskflow INTERFACE IMPORTED GLOBAL)
  set_target_properties(
    Taskflow::Taskflow
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${taskflow_SOURCE_DIR}"
  )
  set(Taskflow_VERSION 4.0.0 CACHE STRING "Taskflow version" FORCE)
  set(Taskflow_FOUND TRUE CACHE BOOL "Taskflow found via FetchContent" FORCE)
endif()
