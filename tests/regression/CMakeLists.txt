# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Improve the way of handling the test cases

dart_add_test("regression" test_Issue1243 test_Issue1243.cpp)

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  dart_add_test("regression" test_Issue1583)
  target_link_libraries(test_Issue1583 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue1596)
  target_link_libraries(test_Issue1596 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue838)
  target_link_libraries(test_Issue838 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue892)
  target_link_libraries(test_Issue892 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue895)
  target_link_libraries(test_Issue895 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue986)
  target_link_libraries(test_Issue986 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue1231)
  target_link_libraries(test_Issue1231 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue1445)
  target_link_libraries(test_Issue1445 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue1193)
  target_link_libraries(test_Issue1193 ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)

  dart_add_test("regression" test_Issue1184)
  target_link_libraries(test_Issue1184 ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision AND TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("regression" test_Issue1234)
  target_link_libraries(test_Issue1234
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
  )

  dart_add_test("regression" test_Issue1624)
  target_link_libraries(test_Issue1624
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
  )

endif()
