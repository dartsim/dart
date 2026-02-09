# Copyright (c) 2011, The DART development contributors

if(TARGET dart-utils)
  dart_build_tests(
    TYPE unit
    COMPONENT_NAME utils
    TARGET_PREFIX UNIT_utils
    LINK_LIBRARIES dart-utils
    GLOB_SOURCES
  )
endif()
