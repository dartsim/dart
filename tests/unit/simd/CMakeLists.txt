# Copyright (c) 2011, The DART development contributors

# Note: All SIMD tests link Eigen because simd.hpp includes geometry types
dart_build_tests(
  TYPE unit
  COMPONENT_NAME simd
  TARGET_PREFIX UNIT_simd
  LINK_LIBRARIES dart-simd Eigen3::Eigen
  GLOB_SOURCES
)
