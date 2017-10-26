dart_add_test("regression" test_Issue000Template test_Issue000Template.cpp)

if(TARGET dart-utils-urdf)

  dart_add_test("regression" test_Issue838)
  target_link_libraries(test_Issue838 dart-utils-urdf)

  dart_add_test("regression" test_Issue895)

endif()
