# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name dynamics)

# Add component
dart_add_component(
  PROJECT_NAME
    ${PROJECT_NAME}
  PROJECT_VERSION_MAJOR
    ${DART_MAJOR_VERSION}
  PROJECT_SOURCE_DIR
    ${DART_SOURCE_DIR}
  PROJECT_BINARY_DIR
    ${DART_BINARY_DIR}
  COMPONENT_NAME
    ${component_name}
  COMPONENT_DEPENDENCIES
    common math optimizer collision
  COMPONENT_DEPENDENCY_PACKAGES
    assimp octomap
  TARGET_LINK_LIBRARIES_PUBLIC
    assimp octomap
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  SUB_DIRECTORIES
    detail dart fcl bullet ode
  GENERATE_META_HEADER
  FORMAT_CODE
)
