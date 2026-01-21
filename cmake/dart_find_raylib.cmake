# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(raylib CONFIG QUIET)

if(NOT raylib_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(raylib QUIET IMPORTED_TARGET raylib)
  endif()
endif()

if(raylib_FOUND AND NOT TARGET raylib::raylib)
  if(TARGET raylib)
    add_library(raylib::raylib ALIAS raylib)
  elseif(TARGET PkgConfig::raylib)
    add_library(raylib::raylib ALIAS PkgConfig::raylib)
  endif()
endif()

