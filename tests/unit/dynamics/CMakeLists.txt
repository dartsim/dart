# Copyright (c) 2011-2025, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME dynamics
  TARGET_PREFIX UNIT_dynamics
  LINK_LIBRARIES dart
  GLOB_SOURCES
)
