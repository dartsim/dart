# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(ODE QUIET CONFIG NAMES ODE ode)

if(ODE_FOUND AND NOT TARGET ODE::ODE)
  add_library(ODE::ODE INTERFACE IMPORTED)
  set(_ode_link_libs "${ODE_LIBRARIES}")
  if(TARGET ode::ode)
    list(APPEND _ode_link_libs ode::ode)
  elseif(TARGET ODE::ODE_double)
    list(APPEND _ode_link_libs ODE::ODE_double)
  elseif(TARGET ODE::ODE_single)
    list(APPEND _ode_link_libs ODE::ODE_single)
  endif()
  list(REMOVE_DUPLICATES _ode_link_libs)
  set_target_properties(
    ODE::ODE
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}" INTERFACE_LINK_LIBRARIES "${_ode_link_libs}"
  )
endif()

if(NOT ODE_FOUND AND NOT ode_FOUND)
  find_package(ODE 0.16.2 QUIET MODULE)

  if(ODE_FOUND AND NOT TARGET ODE::ODE)
    add_library(ODE::ODE INTERFACE IMPORTED)
    set_target_properties(
      ODE::ODE
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}" INTERFACE_LINK_LIBRARIES "${ODE_LIBRARIES}"
    )
  endif()
endif()
