# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(NLOPT 2.4.1 QUIET)

if(NLOPT_FOUND AND NOT TARGET NLOPT::nlopt)
  add_library(NLOPT::nlopt INTERFACE IMPORTED)
  set_target_properties(NLOPT::nlopt PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${NLOPT_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${NLOPT_LIBRARIES}"
  )
endif()
