# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(ccd 2.0 REQUIRED)

# Set target ccd if not set
# Upstream provides the target since 2.1
if((CCD_FOUND OR ccd_FOUND) AND NOT TARGET ccd)
  add_library(ccd INTERFACE IMPORTED)
  set_target_properties(ccd PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CCD_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${CCD_LIBRARIES}"
  )
endif()
