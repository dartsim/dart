# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(assimp 5.2.0 REQUIRED MODULE)

# Set target assimp if not set
if((ASSIMP_FOUND OR assimp_FOUND) AND NOT TARGET assimp)
  add_library(assimp INTERFACE IMPORTED)
  set_target_properties(assimp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ASSIMP_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${ASSIMP_LIBRARIES}"
  )
endif()
