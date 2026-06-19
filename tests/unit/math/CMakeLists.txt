# Copyright (c) 2011, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME math
  TARGET_PREFIX UNIT_math
  LINK_LIBRARIES dart
  GLOB_SOURCES
)

add_library(UNIT_math_ConvhullCCompat OBJECT test_ConvhullCCompat.c)
target_include_directories(
  UNIT_math_ConvhullCCompat
  PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR}
)
target_compile_options(
  UNIT_math_ConvhullCCompat
  PRIVATE
    $<$<AND:$<COMPILE_LANGUAGE:C>,$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>>:-Wno-error>
)
dart_format_add(test_ConvhullCCompat.c)
