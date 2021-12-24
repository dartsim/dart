# Copyright (c) 2011-2021, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME common
  TARGET_PREFIX UNIT_common
  LINK_LIBRARIES dart
  GLOB_SOURCES
)
