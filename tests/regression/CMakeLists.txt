# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Improve the way of handling the test cases

set(test_type "TEST_REGRESSION")

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  
  dart_build_tests(
    TYPE ${test_type}
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
    SOURCES
      test_Issue1583.cpp
      test_Issue1596.cpp
      test_Issue838.cpp
      test_Issue892.cpp
      test_Issue895.cpp
      test_Issue986.cpp
      test_Issue1231.cpp
      test_Issue1445.cpp
      test_Issue1193.cpp
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision AND TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation)

  dart_build_tests(
    TYPE ${test_type}
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    SOURCES
      test_Issue1184.cpp
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision AND TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_build_tests(
    TYPE ${test_type}
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
    SOURCES
      test_Issue1234.cpp
      test_Issue1243.cpp
      test_Issue1624.cpp
  )

endif()
