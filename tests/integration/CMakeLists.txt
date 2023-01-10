# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Improve the way of handling the test cases

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics)

  dart_build_tests(
    TYPE integration
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
    SOURCES
      test_Aspect.cpp
      test_Concurrency.cpp
      test_Distance.cpp
      test_Frames.cpp
      test_Raycast.cpp
      test_Signal.cpp
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation)

  dart_build_tests(
    TYPE integration
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
    SOURCES
      test_Building.cpp
      test_CollisionGroups.cpp
      test_Constraint.cpp
      test_ConstraintSolver.cpp
      test_ContactConstraint.cpp
      test_ForceDependentSlip.cpp
      test_Friction.cpp
      test_GenericJoints.cpp
      test_Inertia.cpp
      test_Lemke.cpp
      test_LocalResourceRetriever.cpp
      test_NameManagement.cpp
      test_Optimizer.cpp
      test_ScrewJoint.cpp
      test_Subscriptions.cpp
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  dart_build_tests(
    TYPE integration
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-io
    SOURCES
      test_CompositeResourceRetriever.cpp
      test_DartResourceRetriever.cpp
      test_FileInfoWorld.cpp
      test_InverseKinematics.cpp
      test_MjcfParser.cpp
      test_PackageResourceRetriever.cpp
      test_SdfParser.cpp
      test_SkelParser.cpp
      test_VskParser.cpp
  )

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
