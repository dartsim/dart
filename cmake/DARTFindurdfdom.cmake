# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(urdfdom QUIET)

if(urdfdom_FOUND AND NOT TARGET urdfdom)
  add_library(urdfdom INTERFACE IMPORTED)
  set_target_properties(urdfdom PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${urdfdom_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${urdfdom_LIBRARIES}"
  )
endif()

