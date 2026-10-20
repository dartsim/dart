# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")

set(DART_ODE_HAS_LIBCCD_BOX_CYL 0)
set(_dart_use_internal_ode FALSE)

if(NOT DART_USE_SYSTEM_ODE)
  if(TARGET ODE)
    if(NOT TARGET ODE::ODE)
      add_library(ODE::ODE ALIAS ODE)
    endif()
    set(ODE_FOUND TRUE)
    set(ode_FOUND TRUE)
    if(NOT ODE_VERSION)
      get_target_property(
        _dart_ode_target_includes
        ODE
        INTERFACE_INCLUDE_DIRECTORIES
      )
      set(_dart_ode_include_dirs ${_dart_ode_target_includes})
      if(DART_ODE_SOURCE_DIR)
        list(APPEND _dart_ode_include_dirs "${DART_ODE_SOURCE_DIR}/include")
      endif()
      dart_read_header_version(ODE_VERSION ode/version.h dODE_VERSION ${_dart_ode_include_dirs})
    endif()
    dart_check_package_version(ODE 0.16.2)
    set(ode_FOUND ${ODE_FOUND})
    set(_dart_use_internal_ode TRUE)
  endif()
endif()

if(NOT _dart_use_internal_ode)
  find_package(ODE QUIET CONFIG NAMES ODE ode)
  set(_dart_ode_config_found FALSE)
  if(ODE_FOUND OR ode_FOUND)
    set(_dart_ode_config_found TRUE)
    if(NOT ODE_VERSION AND ode_VERSION)
      set(ODE_VERSION "${ode_VERSION}")
    endif()
    if(NOT ODE_VERSION)
      set(_dart_ode_include_dirs ${ODE_INCLUDE_DIRS})
      if(TARGET ODE::ODE)
        get_target_property(
          _dart_ode_target_includes
          ODE::ODE
          INTERFACE_INCLUDE_DIRECTORIES
        )
        list(APPEND _dart_ode_include_dirs ${_dart_ode_target_includes})
      endif()
      dart_read_header_version(ODE_VERSION ode/version.h dODE_VERSION ${_dart_ode_include_dirs})
    endif()
    dart_check_package_version(ODE 0.16.2)
    set(ode_FOUND ${ODE_FOUND})
  endif()

  if(NOT _dart_ode_config_found)
    find_package(ODE 0.16.2 QUIET MODULE)

    if(ODE_FOUND AND NOT TARGET ODE::ODE)
      add_library(ODE::ODE INTERFACE IMPORTED)
      set_target_properties(
        ODE::ODE
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
          INTERFACE_LINK_LIBRARIES "${ODE_LIBRARIES}"
          INTERFACE_COMPILE_OPTIONS "${ODE_DEFINITIONS}"
      )
    endif()
  endif()
  unset(_dart_ode_config_found)
endif()

if(ODE_FOUND OR ode_FOUND)
  set(_dart_ode_has_libccd_box_cyl 0)
  set(_dart_ode_defs "")
  if(TARGET ODE::ODE)
    get_target_property(_dart_ode_defs ODE::ODE INTERFACE_COMPILE_DEFINITIONS)
  endif()
  if(DEFINED ODE_DEFINITIONS)
    list(APPEND _dart_ode_defs ${ODE_DEFINITIONS})
  endif()

  if(NOT _dart_use_internal_ode)
    set(_dart_ode_double FALSE)
    foreach(_dart_ode_def IN LISTS _dart_ode_defs)
      if(_dart_ode_def MATCHES "^(-D)?d(IDE)?DOUBLE(=1)?$")
        set(_dart_ode_double TRUE)
      endif()
    endforeach()
    if(NOT _dart_ode_double)
      include(CMakePushCheckState)
      cmake_push_check_state(RESET)
      set(_dart_ode_try_compile_target_type "${CMAKE_TRY_COMPILE_TARGET_TYPE}")
      set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
      set(CMAKE_REQUIRED_LIBRARIES ODE::ODE)
      set(
        _dart_ode_precision_source
        "
#include <ode/odeconfig.h>
#if !defined(dDOUBLE) || defined(dSINGLE)
#error DART requires double-precision ODE
#endif
int main(void) { return 0; }
"
      )
      get_property(_dart_ode_languages GLOBAL PROPERTY ENABLED_LANGUAGES)
      unset(_dart_ode_double CACHE)
      unset(_dart_ode_double)
      if("CXX" IN_LIST _dart_ode_languages)
        include(CheckCXXSourceCompiles)
        check_cxx_source_compiles(
          "${_dart_ode_precision_source}"
          _dart_ode_double
        )
      elseif("C" IN_LIST _dart_ode_languages)
        include(CheckCSourceCompiles)
        check_c_source_compiles(
          "${_dart_ode_precision_source}"
          _dart_ode_double
        )
      endif()
      if(_dart_ode_try_compile_target_type)
        set(
          CMAKE_TRY_COMPILE_TARGET_TYPE
          "${_dart_ode_try_compile_target_type}"
        )
      else()
        unset(CMAKE_TRY_COMPILE_TARGET_TYPE)
      endif()
      unset(_dart_ode_try_compile_target_type)
      cmake_pop_check_state()
      unset(_dart_ode_precision_source)
      unset(_dart_ode_languages)
    endif()
    if(NOT _dart_ode_double)
      message(
        FATAL_ERROR
        "DART requires double-precision ODE (dDOUBLE). Install a double-precision ODE package or configure DART with DART_USE_SYSTEM_ODE=OFF. A C or CXX language is required to check ODE headers when the package provides no precision definition."
      )
    endif()
    unset(_dart_ode_double CACHE)
    unset(_dart_ode_double)
  endif()

  foreach(_dart_ode_def ${_dart_ode_defs})
    if(_dart_ode_def MATCHES "dLIBCCD_BOX_CYL")
      set(_dart_ode_has_libccd_box_cyl 1)
      break()
    endif()
  endforeach()

  if(NOT _dart_ode_has_libccd_box_cyl)
    set(_dart_ode_includes "")
    if(TARGET ODE::ODE)
      get_target_property(
        _dart_ode_includes
        ODE::ODE
        INTERFACE_INCLUDE_DIRECTORIES
      )
    endif()
    if(NOT _dart_ode_includes AND ODE_INCLUDE_DIRS)
      set(_dart_ode_includes "${ODE_INCLUDE_DIRS}")
    endif()

    if(_dart_ode_includes)
      set(CMAKE_REQUIRED_INCLUDES "${_dart_ode_includes}")
    endif()

    set(
      _dart_ode_probe_source
      "
#include <ode/odeconfig.h>
#ifndef dLIBCCD_BOX_CYL
#error dLIBCCD_BOX_CYL not defined
#endif
int main(void) { return 0; }
"
    )
    get_property(_dart_ode_languages GLOBAL PROPERTY ENABLED_LANGUAGES)
    if("CXX" IN_LIST _dart_ode_languages)
      include(CheckCXXSourceCompiles)
      check_cxx_source_compiles(
        "${_dart_ode_probe_source}"
        _dart_ode_has_libccd_box_cyl_macro
      )
    elseif("C" IN_LIST _dart_ode_languages)
      include(CheckCSourceCompiles)
      check_c_source_compiles(
        "${_dart_ode_probe_source}"
        _dart_ode_has_libccd_box_cyl_macro
      )
    endif()
    unset(_dart_ode_languages)
    unset(_dart_ode_probe_source)

    unset(CMAKE_REQUIRED_INCLUDES)
    unset(_dart_ode_includes)

    if(_dart_ode_has_libccd_box_cyl_macro)
      set(_dart_ode_has_libccd_box_cyl 1)
    endif()

    unset(_dart_ode_has_libccd_box_cyl_macro)
  endif()

  set(
    DART_ODE_HAS_LIBCCD_BOX_CYL
    ${_dart_ode_has_libccd_box_cyl}
    CACHE INTERNAL
    "Whether ODE supports libccd box-cylinder contacts"
    FORCE
  )
  unset(_dart_ode_has_libccd_box_cyl)
  unset(_dart_ode_defs)
endif()

unset(_dart_use_internal_ode)
