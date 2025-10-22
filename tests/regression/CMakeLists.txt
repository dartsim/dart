# Copyright (c) 2011-2025, The DART development contributors

dart_build_tests(
  TYPE regression
  TARGET_PREFIX REGRESSION
  LINK_LIBRARIES dart
  SOURCES
    test_Issue000Template.cpp
    test_Issue1243.cpp
)

if(TARGET dart-utils)
  dart_build_tests(
    TYPE regression
    TARGET_PREFIX REGRESSION
    LINK_LIBRARIES dart dart-utils
    SOURCES
      test_Issue1193.cpp
      test_Issue1583.cpp
      test_Issue1596.cpp
  )
endif()

if(TARGET dart-utils-urdf)
  dart_build_tests(
    TYPE regression
    TARGET_PREFIX REGRESSION
    LINK_LIBRARIES dart dart-utils-urdf
    SOURCES
      test_Issue1231.cpp
      test_Issue1445.cpp
      test_Issue838.cpp
      test_Issue892.cpp
      test_Issue895.cpp
      test_Issue986.cpp
  )
endif()

if(TARGET dart-collision-bullet)
  dart_build_tests(
    TYPE regression
    TARGET_PREFIX REGRESSION
    LINK_LIBRARIES dart dart-collision-bullet
    SOURCES
      test_Issue1184.cpp
  )
endif()

if(TARGET dart-collision-bullet AND TARGET dart-collision-ode)
  dart_build_tests(
    TYPE regression
    TARGET_PREFIX REGRESSION
    LINK_LIBRARIES dart dart-collision-bullet dart-collision-ode dart-utils
    SOURCES
      test_Issue1234.cpp
  )

  dart_build_tests(
    TYPE regression
    TARGET_PREFIX REGRESSION
    LINK_LIBRARIES dart dart-collision-ode dart-utils
    SOURCES
      test_Issue1624.cpp
  )
endif()
