# Copyright (c) 2011, The DART development contributors

if(TARGET dart-gui)
  dart_build_tests(
    TYPE unit
    COMPONENT_NAME gui
    TARGET_PREFIX UNIT_gui
    LINK_LIBRARIES dart-gui dart-external-lodepng
    GLOB_SOURCES
  )
endif()
