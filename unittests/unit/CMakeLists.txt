dart_add_test("unit" test_Aspect)
dart_add_test("unit" test_CollisionGroups)
dart_add_test("unit" test_ContactConstraint)
dart_add_test("unit" test_Factory)
dart_add_test("unit" test_GenericJoints)
dart_add_test("unit" test_Geometry)
dart_add_test("unit" test_Lemke)
dart_add_test("unit" test_LocalResourceRetriever)
dart_add_test("unit" test_Math)
dart_add_test("unit" test_Optimizer)
dart_add_test("unit" test_Random)
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

if(TARGET dart-io)

  dart_add_test("unit" test_CompositeResourceRetriever)
  target_link_libraries(test_CompositeResourceRetriever dart-io)

  dart_add_test("unit" test_DartResourceRetriever)
  target_link_libraries(test_DartResourceRetriever dart-io)

  dart_add_test("unit" test_FileInfoWorld)
  target_link_libraries(test_FileInfoWorld dart-io)

  dart_add_test("unit" test_PackageResourceRetriever)
  target_link_libraries(test_PackageResourceRetriever dart-io)

  dart_add_test("unit" test_SdfParser)
  target_link_libraries(test_SdfParser dart-io)

  dart_add_test("unit" test_SkelParser)
  target_link_libraries(test_SkelParser dart-io)

  dart_add_test("unit" test_VskParser)
  target_link_libraries(test_VskParser dart-io)

endif()

if(TARGET dart-io-urdf)
  dart_add_test("unit" test_DartLoader)
  target_link_libraries(test_DartLoader dart-io-urdf)

  dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

  dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(GeneratedWamIkFast PRIVATE -w)

  dart_add_test("unit" test_IkFast)
  target_link_libraries(test_IkFast dart-io-urdf)
  add_dependencies(test_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
endif()

foreach(collision_engine
    dart-collision-bullet
    dart-collision-ode)

  if(TARGET ${collision_engine})
    target_link_libraries(test_CollisionGroups ${collision_engine})
  endif()

endforeach()
