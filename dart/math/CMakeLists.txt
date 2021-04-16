# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name math)

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
    common
  COMPONENT_DEPENDENCY_PACKAGES
    Boost Eigen3
  TARGET_LINK_LIBRARIES_PUBLIC
    Eigen3::Eigen
  TARGET_LINK_LIBRARIES_PRIVATE
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-external-odelcpsolver
  TARGET_LINK_OPTIONS_PUBLIC
    ${link_options_public}
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  TARGET_COMPILE_OPTIONS_PUBLIC
    ${compile_options_public}
  SUB_DIRECTORIES
    detail
  GENERATE_META_HEADER
  FORMAT_CODE
)
