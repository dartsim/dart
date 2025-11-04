# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(Eigen3 REQUIRED CONFIG)

if (Eigen3_VERSION VERSION_LESS 3.4)
  message(FATAL_ERROR "Eigen version>=3.4 is required, but found ${Eigen3_VERSION}")
endif()
