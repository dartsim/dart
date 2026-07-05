dart_add_test("unit" test_Contact test_Contact.cpp)
dart_add_test("unit" test_DARTCollisionDetector test_DARTCollisionDetector.cpp)
dart_add_test("unit" test_FCLCollisionObject test_FCLCollisionObject.cpp)

dart_add_test(
  "unit"
  test_ConvexMeshShapeCollision
  test_ConvexMeshShapeCollision.cpp
)
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_ConvexMeshShapeCollision ${collision_engine})
  endif()
endforeach()

if(HAVE_BULLET)
  dart_add_test("unit" test_BulletContact test_BulletContact.cpp)
  target_link_libraries(test_BulletContact dart-collision-bullet)
endif()

add_subdirectory(native)
