# Copyright (c) 2011, The DART development contributors

# Note: dart-simd propagates Eigen because Simd.hpp includes geometry types.
dart_build_tests(
  TYPE unit
  COMPONENT_NAME simd
  TARGET_PREFIX UNIT_simd
  LINK_LIBRARIES dart-simd
  GLOB_SOURCES
)
