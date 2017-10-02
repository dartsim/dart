dart_add_test("unit" test_AffineExpression)
dart_add_test("unit" test_Aspect)
dart_add_test("unit" test_ContactConstraint)
dart_add_test("unit" test_Factory)
dart_add_test("unit" test_GenericJoints)
dart_add_test("unit" test_Geometry)
dart_add_test("unit" test_Lemke)
dart_add_test("unit" test_LocalResourceRetriever)
dart_add_test("unit" test_Math)
dart_add_test("unit" test_Optimizer)
dart_add_test("unit" test_ScrewJoint)
dart_add_test("unit" test_Signal)
dart_add_test("unit" test_Subscriptions)
dart_add_test("unit" test_Uri)

if(TARGET dart-optimizer-ipopt)
  target_link_libraries(test_Optimizer dart-optimizer-ipopt)
endif()

if(TARGET dart-optimizer-nlopt)
  target_link_libraries(test_Optimizer dart-optimizer-nlopt)
endif()

if(TARGET dart-utils)

  dart_add_test("unit" test_CompositeResourceRetriever)
  target_link_libraries(test_CompositeResourceRetriever dart-utils)

  dart_add_test("unit" test_DartResourceRetriever)
  target_link_libraries(test_DartResourceRetriever dart-utils)

  dart_add_test("unit" test_FileInfoWorld)
  target_link_libraries(test_FileInfoWorld dart-utils)

  dart_add_test("unit" test_PackageResourceRetriever)
  target_link_libraries(test_PackageResourceRetriever dart-utils)

  dart_add_test("unit" test_SdfParser)
  target_link_libraries(test_SdfParser dart-utils)

  dart_add_test("unit" test_SkelParser)
  target_link_libraries(test_SkelParser dart-utils)

  dart_add_test("unit" test_VskParser)
  target_link_libraries(test_VskParser dart-utils)

endif()

if(TARGET dart-utils-urdf)
  dart_add_test("unit" test_DartLoader)
  target_link_libraries(test_DartLoader dart-utils-urdf)
endif()

if(TARGET dart-planning)
  dart_add_test("unit" test_NearestNeighbor)
  target_link_libraries(test_NearestNeighbor dart-planning)
endif()
