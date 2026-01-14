# Copyright (c) 2011-2025, The DART development contributors

if(TARGET dart-utils)
  dart_build_tests(
    TYPE unit
    COMPONENT_NAME utils
    TARGET_PREFIX UNIT_utils
    LINK_LIBRARIES dart-utils tinyxml2::tinyxml2
    GLOB_SOURCES
  )
endif()
