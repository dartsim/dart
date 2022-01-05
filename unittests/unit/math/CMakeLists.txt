# Copyright (c) 2011-2022, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME math
  TARGET_PREFIX UNIT_math
  LINK_LIBRARIES dart
  GLOB_SOURCES
)
