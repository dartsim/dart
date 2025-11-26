dart_add_test("unit" test_Contact test_Contact.cpp)
dart_add_test("unit" test_FCLCollisionObject test_FCLCollisionObject.cpp)

if(HAVE_BULLET)
  dart_add_test("unit" test_BulletContact test_BulletContact.cpp)
  target_link_libraries(test_BulletContact dart-collision-bullet)
endif()
