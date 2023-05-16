# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name io)

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
  DEPENDENT_COMPONENTS
    common math optimization collision dynamics simulation
  DEPENDENT_PACKAGES_REQUIRED
    tinyxml2
  DEPENDENT_PACKAGES_OPTIONAL
    CURL
  TARGET_LINK_LIBRARIES_PUBLIC
    tinyxml2::tinyxml2
  TARGET_LINK_LIBRARIES_PUBLIC_OPTIONAL
    CURL::libcurl
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  SUB_DIRECTORIES
    detail mjcf sdf urdf
  GENERATE_EXPORT_HEADER
  GENERATE_META_HEADER
  FORMAT_CODE
)
