# Copyright (c) 2011-2023, The DART development contributors

# TODO(JS): Improve the way of handling the test cases

dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-test
  SOURCES
    test_Aspect.cpp
    test_Building.cpp
    test_Common.cpp
    test_Concurrency.cpp
    test_Constraint.cpp
    test_Frames.cpp
    test_Friction.cpp
    test_InverseKinematics.cpp
    test_NameManagement.cpp
    test_Signal.cpp
)

dart_add_test("integration" test_CollisionGroups)
foreach(collision_engine
  ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  if(TARGET ${collision_engine})
    target_link_libraries(test_CollisionGroups ${collision_engine})
  endif()
endforeach()
dart_add_test("integration" test_ConstraintSolver)
dart_add_test("integration" test_ContactConstraint)
dart_add_test("integration" test_GenericJoints)
dart_add_test("integration" test_Inertia)
dart_add_test("integration" test_Lemke)
dart_add_test("integration" test_LocalResourceRetriever)
dart_add_test("integration" test_Optimizer)
dart_add_test("integration" test_ScrewJoint)
dart_add_test("integration" test_Subscriptions)

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimizer)
  target_link_libraries(test_Optimizer ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimizer)
endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_CompositeResourceRetriever)
  target_link_libraries(test_CompositeResourceRetriever ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_DartResourceRetriever)
  target_link_libraries(test_DartResourceRetriever ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  
  dart_add_test("integration" test_FileInfoWorld)
  target_link_libraries(test_FileInfoWorld ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_PackageResourceRetriever)
  target_link_libraries(test_PackageResourceRetriever ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_MjcfParser)
  target_link_libraries(test_MjcfParser ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  
  dart_add_test("integration" test_SdfParser)
  target_link_libraries(test_SdfParser ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  
  dart_add_test("integration" test_SkelParser)
  target_link_libraries(test_SkelParser ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  if(NOT MSVC) 
    dart_add_test("integration" test_VskParser)
    target_link_libraries(test_VskParser ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  endif()

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  if(NOT MSVC)
    dart_add_test("integration" test_DartLoader)
    target_link_libraries(test_DartLoader ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  endif()

  if(BUILD_SHARED_LIBS)
    dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
    target_link_libraries(SharedLibraryWamIkFast PUBLIC ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
    target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
    target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

    dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
    target_link_libraries(GeneratedWamIkFast PUBLIC ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
    target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
    target_compile_options(GeneratedWamIkFast PRIVATE -w)

    dart_add_test("integration" test_IkFast)
    target_link_libraries(test_IkFast ${PROJECT_NAME}${DART_MAJOR_VERSION}-io GeneratedWamIkFast SharedLibraryWamIkFast)
  endif()
endif()

dart_format_add(
  GeneratedWamIkFast.cpp
  SharedLibraryWamIkFast.hpp
  SharedLibraryWamIkFast.cpp
)

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimizer)
  dart_add_test("integration" test_MultiObjectiveOptimization)
  target_link_libraries(test_MultiObjectiveOptimization ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimizer)
endif()

dart_add_test("integration" test_Distance)
if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  target_link_libraries(test_Distance ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
endif()

dart_add_test("integration" test_Raycast)
if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  target_link_libraries(test_Raycast ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  # This test doesn't work with FCL because it converts simple shapes to meshes, which makes
  # it difficult to come up with correct test expectations.
  dart_add_test("integration" test_ForceDependentSlip)
  target_link_libraries(test_ForceDependentSlip ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_Collision)
  target_link_libraries(test_Collision ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
    target_link_libraries(test_Collision ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  endif()
  if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
    target_link_libraries(test_Collision ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  endif()

  dart_add_test("integration" test_Dynamics)
  target_link_libraries(test_Dynamics ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_Joints)
  target_link_libraries(test_Joints ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_JointForceTorque)
  target_link_libraries(test_JointForceTorque ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_Skeleton)
  target_link_libraries(test_Skeleton ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_MetaSkeleton)
  target_link_libraries(test_MetaSkeleton ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_SoftDynamics)
  target_link_libraries(test_SoftDynamics ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_add_test("integration" test_World)
  target_link_libraries(test_World ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
    target_link_libraries(test_World ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision)
  endif()

  if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
    dart_add_test("integration" test_ForwardKinematics)
    target_link_libraries(test_ForwardKinematics ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
  endif()

endif()
