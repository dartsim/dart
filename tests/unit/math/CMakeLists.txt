# Copyright (c) 2011-2025, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME math
  TARGET_PREFIX UNIT_math
  LINK_LIBRARIES dart
  GLOB_SOURCES
)

# Add baseline directory to include path for comparison tests
target_include_directories(UNIT_math_Convhull
  PRIVATE
  ${CMAKE_SOURCE_DIR}/tests
)

# Suppress warnings for baseline code (third-party C code used for comparison)
target_compile_options(UNIT_math_Convhull
  PRIVATE
  -Wno-unused-parameter
  -Wno-unused-but-set-variable
)
