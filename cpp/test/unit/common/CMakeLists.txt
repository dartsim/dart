# Copyright (c) 2011-2022, The DART development contributors

dart_build_unit_tests(
  COMPONENT_NAME common
  TARGET_PREFIX UNIT_common
  LINK_LIBRARIES
    ${PROJECT_NAME}${DART_VERSION_MAJOR}-test-common
    ${PROJECT_NAME}${DART_VERSION_MAJOR}-test-math
)
