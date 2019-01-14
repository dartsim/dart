# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(ODE 0.13 QUIET)

if(ODE_FOUND AND NOT TARGET ODE::ODE)
  add_library(ODE::ODE INTERFACE IMPORTED)
  set_target_properties(ODE::ODE PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${ODE_LIBRARIES}"
  )
endif()
