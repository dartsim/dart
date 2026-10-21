# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Upstream assimp 5.3.1 still reports 5.3.0 (Ubuntu 24.04 ships it).
find_package(assimp 5.3.0 QUIET MODULE)
if(NOT ASSIMP_FOUND AND NOT assimp_FOUND)
  # Without pkg-config, take assimp's version from its own CMake package, loaded
  # only now so lookups that already work stay unchanged. Search without a
  # version: a lookup nested in Findassimp.cmake inherits the version request,
  # which assimp 6's version file rejects. Keep the paths the module search used
  # rather than the package's, whose target can name uninstalled files, such as
  # libz.so on Ubuntu 22.04.
  set(_dart_assimp_include_dirs "${ASSIMP_INCLUDE_DIRS}")
  set(_dart_assimp_libraries "${ASSIMP_LIBRARIES}")
  find_package(assimp QUIET CONFIG)
  set(ASSIMP_INCLUDE_DIRS "${_dart_assimp_include_dirs}")
  set(ASSIMP_LIBRARIES "${_dart_assimp_libraries}")
  unset(_dart_assimp_include_dirs)
  unset(_dart_assimp_libraries)
  find_package(assimp 5.3.0 REQUIRED MODULE)
endif()

# Set target assimp if not set
if((ASSIMP_FOUND OR assimp_FOUND) AND NOT TARGET assimp)
  add_library(assimp INTERFACE IMPORTED)
  set_target_properties(
    assimp
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${ASSIMP_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${ASSIMP_LIBRARIES}"
  )
endif()
