# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

if(APPLE)
  find_package(OpenCL 1.2 QUIET)
else()
  find_package(OpenCL QUIET)
endif()
