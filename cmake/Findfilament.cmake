# Copyright (c) 2011-2018, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find filament
#
# This sets the following variables:
# filament_FOUND
# filament_INCLUDE_DIRS
# filament_LIBRARIES
# filament_VERSION

# Include directories
find_path(filament_INCLUDE_DIRS filament/Scene.h
  PATHS "${CMAKE_INSTALL_PREFIX}/include"
)

# Workaround, not sure if this a right way to do
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(CMAKE_LIBRARY_ARCHITECTURE x86_64)
endif()

# Libraries
if(MSVC)
  set(filament_LIBRARIES optimized filaflat debug filaflatd)
else()
  find_library(filament_filabridge_LIBRARY NAMES filabridge)
  find_library(filament_filaflat_LIBRARY NAMES filaflat)
  find_library(filament_filamat_LIBRARY NAMES filamat)
  find_library(filament_utils_LIBRARY NAMES utils)
  find_library(filament_filament_LIBRARY NAMES filament)
  find_library(filament_bluevk_LIBRARY NAMES bluevk)
  find_library(filament_bluegl_LIBRARY NAMES bluegl)
endif()
set(filament_LIBRARIES
  ${filament_filabridge_LIBRARY}
  ${filament_filaflat_LIBRARY}
  ${filament_filamat_LIBRARY}
  ${filament_utils_LIBRARY}
  ${filament_filament_LIBRARY}
  ${filament_bluevk_LIBRARY}
  ${filament_bluegl_LIBRARY}
)

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(filament
  FAIL_MESSAGE  DEFAULT_MSG
  REQUIRED_VARS filament_INCLUDE_DIRS filament_LIBRARIES
)
