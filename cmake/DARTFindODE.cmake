# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(ODE QUIET CONFIG NAMES ODE ode)

if(NOT ODE_FOUND AND NOT ode_FOUND)

  find_package(ODE 0.16 QUIET MODULE)

  if(ODE_FOUND AND NOT TARGET ODE::ODE)
    add_library(ODE::ODE INTERFACE IMPORTED)
    set_target_properties(ODE::ODE PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${ODE_LIBRARIES}"
    )
  endif()

endif()
