# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Eigen 5's version file matches a requested single version only within that
# minor release, and Eigen 3.4's rejects version ranges that span majors, so
# find any Eigen3 package and check the floor here.
find_package(Eigen3 REQUIRED CONFIG)
if(NOT Eigen3_VERSION OR Eigen3_VERSION VERSION_LESS 3.4.0)
  if(NOT Eigen3_VERSION)
    set(Eigen3_VERSION "unknown")
  endif()
  message(
    FATAL_ERROR
    "Eigen3 >= 3.4.0 is required; found version ${Eigen3_VERSION}"
  )
endif()
