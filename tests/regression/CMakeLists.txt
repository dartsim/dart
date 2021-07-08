dart_add_test("regression" test_Issue000Template test_Issue000Template.cpp)

dart_add_test("regression" test_Issue1243 test_Issue1243.cpp)

if(TARGET dart-io)
  dart_add_test("regression" test_Issue1583)
  target_link_libraries(test_Issue1583 dart-io)
endif()

if(TARGET dart-io-urdf)

  dart_add_test("regression" test_Issue838)
  target_link_libraries(test_Issue838 dart-io-urdf)

  dart_add_test("regression" test_Issue892)

  dart_add_test("regression" test_Issue895)

  dart_add_test("regression" test_Issue986)

  dart_add_test("regression" test_Issue1231)

  dart_add_test("regression" test_Issue1445)

endif()

if(TARGET dart-collision-bullet)

  dart_add_test("regression" test_Issue1184)
  target_link_libraries(test_Issue1184 dart-collision-bullet)

endif()

if(TARGET dart-collision-bullet AND TARGET dart-collision-ode)

  dart_add_test("regression" test_Issue1234)
  target_link_libraries(test_Issue1234
    dart-collision-bullet
    dart-collision-ode
    dart-io)

endif()
