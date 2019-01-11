# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(IPOPT 3.11.9 QUIET)

if(IPOPT_FOUND AND NOT TARGET IPOPT::ipopt)
  add_library(IPOPT::ipopt INTERFACE IMPORTED)
  set_target_properties(IPOPT::ipopt PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${IPOPT_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${IPOPT_LIBRARIES}"
  )
endif()
