# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Locate an OpenUSD (pxr) install for the optional dart/io/usd scene loader.
#
# OpenUSD ships a CMake config package (pxrConfig.cmake) that exports the
# per-library imported targets (e.g. usd, usdGeom, sdf, tf) and sets the
# PXR_INCLUDE_DIRS / PXR_LIBRARIES variables. This wrapper keeps the discovery
# snake_case and behind DART_BUILD_IO_USD so the default build, which has no
# OpenUSD dependency, never looks for pxr.

find_package(pxr QUIET CONFIG)

if(pxr_FOUND)
  set(PXR_FOUND TRUE)
  if(DEFINED PXR_INCLUDE_DIRS)
    set(DART_PXR_INCLUDE_DIRS ${PXR_INCLUDE_DIRS})
  endif()
  if(DEFINED PXR_LIBRARIES)
    set(DART_PXR_LIBRARIES ${PXR_LIBRARIES})
  endif()
else()
  set(PXR_FOUND FALSE)
endif()
