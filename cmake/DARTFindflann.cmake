# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(flann 1.8.4 QUIET)

if((FLANN_FOUND OR flann_FOUND) AND NOT TARGET flann)
  add_library(flann INTERFACE IMPORTED)
  set_target_properties(flann PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${FLANN_LIBRARIES}"
  )
endif()
