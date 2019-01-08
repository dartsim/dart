# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(assimp 3.2 REQUIRED)

# Set target assimp if not set
# The target is not imported by upstream until 4.1
if(ASSIMP_FOUND AND NOT TARGET assimp)
  add_library(assimp INTERFACE IMPORTED)
  set_target_properties(assimp PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${ASSIMP_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIES "${ASSIMP_LIBRARIES}"
  )
endif()
