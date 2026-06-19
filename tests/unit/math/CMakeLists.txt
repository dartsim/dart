# Copyright (c) 2011, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME math
  TARGET_PREFIX UNIT_math
  LINK_LIBRARIES dart
  GLOB_SOURCES
)

add_library(UNIT_math_LegacyConvexHullC OBJECT test_LegacyConvexHullC.c)
dart_property_add(DART_unit_TESTS UNIT_math_LegacyConvexHullC)
target_include_directories(
  UNIT_math_LegacyConvexHullC
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(
  UNIT_math_LegacyConvexHullC
  PRIVATE
    $<$<AND:$<COMPILE_LANGUAGE:C>,$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>>:-Wno-error>
)
dart_format_add(test_LegacyConvexHullC.c)
