# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

dart_add_component_sub_directory(
  DEPENDENT_PACKAGES_REQUIRED
    NLOPT
  TARGET_LINK_LIBRARIES_PUBLIC
    NLOPT::nlopt
)
