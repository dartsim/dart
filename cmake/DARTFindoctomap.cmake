# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(octomap 1.8.1 QUIET CONFIG)

if(octomap_FOUND AND NOT TARGET octomap)
  add_library(octomap INTERFACE IMPORTED)
  set_target_properties(octomap PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}"
  )
endif()
