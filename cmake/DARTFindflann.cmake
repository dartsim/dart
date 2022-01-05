# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# The latest tag 1.9.1 doesn't support C++17, which is the minimum
# requirement of DART >= 6.12. It should be resolved once
# https://github.com/flann-lib/flann/issues/476 as mainline already
# has the fix but just not released yet. Until then, the minimum
# required flann version to 1.9.2, which should be correctly updated
# once a new flann release is out.
find_package(flann 1.9.2 QUIET MODULE)

if((FLANN_FOUND OR flann_FOUND) AND NOT TARGET flann)
  add_library(flann INTERFACE IMPORTED)
  set_target_properties(flann PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${FLANN_LIBRARIES}"
  )
endif()
