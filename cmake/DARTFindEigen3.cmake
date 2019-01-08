# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# We intentionally don't specify the required Eigen3 version like
# find_package(Eigen3 3.2.92) because the version file is not provided by
# upstream until 3.3.1.
find_package(Eigen3 REQUIRED)
if(EIGEN3_VERSION_STRING VERSION_LESS 3.2.92)  # 3.3~beta1
  message(FATAL_ERROR "Eigen3 ${EIGEN3_VERSION_STRING} is found but >= 3.2.92
    (3.3~beta1) is required"
  )
endif()

# Set target Eigen3::Eigen if not set
# The target is not imported by upstream until 3.3.2
if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
  )
endif()
