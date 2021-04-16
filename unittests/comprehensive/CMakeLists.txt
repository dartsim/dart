dart_add_test("comprehensive" test_Building)
dart_add_test("comprehensive" test_Common)
dart_add_test("comprehensive" test_Concurrency)
dart_add_test("comprehensive" test_Constraint)
dart_add_test("comprehensive" test_Frames)
dart_add_test("comprehensive" test_Friction)
dart_add_test("comprehensive" test_InverseKinematics)
dart_add_test("comprehensive" test_NameManagement)

if(TARGET dart-optimization-pagmo)
  dart_add_test("comprehensive" test_MultiObjectiveOptimization)
  target_link_libraries(test_MultiObjectiveOptimization dart-optimization-pagmo)
endif()

dart_add_test("comprehensive" test_Distance)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_Distance dart-collision-bullet)
endif()

dart_add_test("comprehensive" test_Raycast)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_Raycast dart-collision-bullet)
endif()

if(TARGET dart-collision-ode)
  # This test doesn't work with FCL because it converts simple shapes to meshes, which makes
  # it difficult to come up with correct test expectations.
  dart_add_test("comprehensive" test_ForceDependentSlip)
  target_link_libraries(test_ForceDependentSlip dart-collision-ode)
endif()

if(TARGET dart-io)

  dart_add_test("comprehensive" test_Collision)
  target_link_libraries(test_Collision dart-io)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_Collision dart-collision-bullet)
  endif()
  if(TARGET dart-collision-ode)
    target_link_libraries(test_Collision dart-collision-ode)
  endif()

  dart_add_test("comprehensive" test_Dynamics)
  target_link_libraries(test_Dynamics dart-io)

  dart_add_test("comprehensive" test_Joints)
  target_link_libraries(test_Joints dart-io)

  dart_add_test("comprehensive" test_Skeleton)
  target_link_libraries(test_Skeleton dart-io)

  dart_add_test("comprehensive" test_MetaSkeleton)
  target_link_libraries(test_MetaSkeleton dart-io)

  dart_add_test("comprehensive" test_SoftDynamics)
  target_link_libraries(test_SoftDynamics dart-io)

  dart_add_test("comprehensive" test_World)
  target_link_libraries(test_World dart-io)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_World dart-collision-bullet)
  endif()

  if(TARGET dart-io-urdf)
    dart_add_test("comprehensive" test_ForwardKinematics)
    target_link_libraries(test_ForwardKinematics dart-io-urdf)
  endif()

endif()
