# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Add definitions for the IpOpt version
if(IPOPT_VERSION)
  string(REPLACE "." ";" ipopt_version_list ${IPOPT_VERSION})
  list(GET ipopt_version_list 0 IPOPT_VERSION_MAJOR)
  list(GET ipopt_version_list 1 IPOPT_VERSION_MINOR)
  list(GET ipopt_version_list 2 IPOPT_VERSION_PATCH)
  target_compile_definitions(${target_name}
    PUBLIC
      IPOPT_VERSION_MAJOR=${IPOPT_VERSION_MAJOR}
      IPOPT_VERSION_MINOR=${IPOPT_VERSION_MINOR}
      IPOPT_VERSION_PATCH=${IPOPT_VERSION_PATCH}
  )
  set(target_compile_definitions_public
    IPOPT_VERSION_MAJOR=${IPOPT_VERSION_MAJOR}
    IPOPT_VERSION_MINOR=${IPOPT_VERSION_MINOR}
    IPOPT_VERSION_PATCH=${IPOPT_VERSION_PATCH}
  )
endif()

dart_add_component_sub_directory(
  COMPONENT_DEPENDENCY_PACKAGES
    IPOPT
  TARGET_LINK_LIBRARIES_PUBLIC
    IPOPT::ipopt
  TARGET_COMPILE_DEFINITIONS_PUBLIC
    ${target_compile_definitions_public}
)
