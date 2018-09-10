# Copyright (c) 2011-2018, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find PAGMO
#
# This sets the following variables:
# PAGMO_FOUND
# PAGMO_INCLUDE_DIRS

# Include directories
find_path(PAGMO_INCLUDE_DIRS
  NAMES pagmo/pagmo.hpp
  PATHS "${CMAKE_INSTALL_PREFIX}/include"
)

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PAGMO
  FAIL_MESSAGE  DEFAULT_MSG
  REQUIRED_VARS PAGMO_INCLUDE_DIRS
)
