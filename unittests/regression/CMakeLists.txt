dart_add_test("regression" test_Issue000Template test_Issue000Template.cpp)

if(TARGET dart-io-urdf)

  dart_add_test("regression" test_Issue838)
  target_link_libraries(test_Issue838 dart-io-urdf)

  dart_add_test("regression" test_Issue895)

  dart_add_test("regression" test_Issue986)

endif()

if(TARGET dart-collision-bullet)

  dart_add_test("regression" test_Issue1184)
  target_link_libraries(test_Issue1184 dart-collision-bullet)

endif()
