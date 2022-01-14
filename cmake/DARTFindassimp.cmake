# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(assimp REQUIRED MODULE)

# Manually check version because the upstream version compatibility policy
# doesn't allow different major number while DART is compatible any version
# greater than or equal to 4.1.
set(DART_ASSIMP_VERSION 4.1)
if(ASSIMP_VERSION AND ASSIMP_VERSION VERSION_LESS ${DART_ASSIMP_VERSION})
  message(SEND_ERROR "Found Assimp ${ASSIMP_VERSION}, but Assimp >= ${DART_ASSIMP_VERSION}
    is required"
  )
endif()

# Set target assimp if not set
if((ASSIMP_FOUND OR assimp_FOUND) AND NOT TARGET assimp)
  add_library(assimp INTERFACE IMPORTED)
  set_target_properties(assimp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ASSIMP_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${ASSIMP_LIBRARIES}"
  )
endif()
