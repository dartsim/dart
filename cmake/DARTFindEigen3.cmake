# Copyright (c) 2011-2018, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(Eigen3 3.0.5 REQUIRED)

# Set target Eigen3::Eigen if not set
if(EIGEN3_FOUND AND NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
  )
endif()
