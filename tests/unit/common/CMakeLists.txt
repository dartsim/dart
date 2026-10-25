# Copyright (c) 2011, The DART development contributors

dart_build_tests(
  TYPE unit
  COMPONENT_NAME common
  TARGET_PREFIX UNIT_common
  LINK_LIBRARIES dart
  GLOB_SOURCES
)

# test_StringType reads a header static from C++17 code and from
# StringTypeCxx20.cpp, which is compiled as C++20 and listed after it so that
# the link keeps the C++17 copy of the static.
target_sources(UNIT_common_StringType PRIVATE StringTypeCxx20.cpp)
if(NOT MSVC)
  set_source_files_properties(
    StringTypeCxx20.cpp
    PROPERTIES COMPILE_OPTIONS -std=c++20
  )
endif()
dart_format_add(StringTypeCxx20.cpp)
