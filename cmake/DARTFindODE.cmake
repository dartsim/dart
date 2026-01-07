# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

set(DART_ODE_HAS_LIBCCD_BOX_CYL 0)
set(_dart_use_internal_ode FALSE)

if(NOT DART_USE_SYSTEM_ODE)
  if(TARGET ODE)
    if(NOT TARGET ODE::ODE)
      add_library(ODE::ODE ALIAS ODE)
    endif()
    set(ODE_FOUND TRUE)
    set(ode_FOUND TRUE)
    set(_dart_use_internal_ode TRUE)
  endif()
endif()

if(NOT _dart_use_internal_ode)
  find_package(ODE QUIET CONFIG NAMES ODE ode)

  if(NOT ODE_FOUND AND NOT ode_FOUND)

    find_package(ODE 0.16.2 QUIET MODULE)

    if(ODE_FOUND AND NOT TARGET ODE::ODE)
      add_library(ODE::ODE INTERFACE IMPORTED)
      set_target_properties(ODE::ODE PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${ODE_LIBRARIES}"
      )
    endif()

  endif()
endif()

if(ODE_FOUND OR ode_FOUND OR TARGET ODE::ODE)
  set(_dart_ode_has_libccd_box_cyl 0)
  set(_dart_ode_defs "")
  if(TARGET ODE::ODE)
    get_target_property(_dart_ode_defs ODE::ODE
      INTERFACE_COMPILE_DEFINITIONS)
  endif()
  if(DEFINED ODE_DEFINITIONS)
    list(APPEND _dart_ode_defs ${ODE_DEFINITIONS})
  endif()

  foreach(_dart_ode_def ${_dart_ode_defs})
    if(_dart_ode_def MATCHES "dLIBCCD_BOX_CYL")
      set(_dart_ode_has_libccd_box_cyl 1)
      break()
    endif()
  endforeach()

  if(NOT _dart_ode_has_libccd_box_cyl)
    include(CheckCSourceCompiles)

    set(_dart_ode_includes "")
    if(TARGET ODE::ODE)
      get_target_property(_dart_ode_includes ODE::ODE
        INTERFACE_INCLUDE_DIRECTORIES)
    endif()
    if(NOT _dart_ode_includes AND ODE_INCLUDE_DIRS)
      set(_dart_ode_includes "${ODE_INCLUDE_DIRS}")
    endif()

    if(_dart_ode_includes)
      set(CMAKE_REQUIRED_INCLUDES "${_dart_ode_includes}")
    endif()

    check_c_source_compiles("
#include <ode/odeconfig.h>
#ifndef dLIBCCD_BOX_CYL
#error dLIBCCD_BOX_CYL not defined
#endif
int main(void) { return 0; }
" _dart_ode_has_libccd_box_cyl_macro)

    unset(CMAKE_REQUIRED_INCLUDES)
    unset(_dart_ode_includes)

    if(_dart_ode_has_libccd_box_cyl_macro)
      set(_dart_ode_has_libccd_box_cyl 1)
    endif()

    unset(_dart_ode_has_libccd_box_cyl_macro)
  endif()

  set(DART_ODE_HAS_LIBCCD_BOX_CYL ${_dart_ode_has_libccd_box_cyl}
    CACHE INTERNAL "Whether ODE supports libccd box-cylinder contacts" FORCE)
  unset(_dart_ode_has_libccd_box_cyl)
  unset(_dart_ode_defs)
endif()

unset(_dart_use_internal_ode)
