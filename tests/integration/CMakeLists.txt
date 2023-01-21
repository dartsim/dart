# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Improve the way of handling the test cases

set(test_type "TEST_INTEGRATION")

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics)

  set(sources
    test_Aspect.cpp
    test_Concurrency.cpp
    test_Distance.cpp
    test_Frames.cpp
    test_Raycast.cpp
    test_Signal.cpp
  )

  # TODO(JS): Fix the test case on macOS 12
  if(DART_IN_CI OR MSVC)
    list(REMOVE_ITEM sources test_Aspect.cpp)
  endif()

  dart_build_tests(
    TYPE ${test_type}
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
    SOURCES
      ${sources}
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation)

  dart_build_tests(
    TYPE ${test_type}
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
      test_LocalResourceRetriever.cpp
      test_NameManagement.cpp
      test_Optimizer.cpp
      test_ScrewJoint.cpp
      test_Subscriptions.cpp
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)

  set(sources
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

  dart_build_tests(
    TYPE ${test_type}
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
    SOURCES
      ${sources}
  )

endif()

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io AND NOT MSVC)

  if(BUILD_SHARED_LIBS)
    dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
    target_link_libraries(SharedLibraryWamIkFast PUBLIC ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
    target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
    target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

    dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
    target_link_libraries(GeneratedWamIkFast PUBLIC ${PROJECT_NAME}${DART_MAJOR_VERSION}-io)
    target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
    target_compile_options(GeneratedWamIkFast PRIVATE -w)

    dart_build_tests(
      TYPE ${test_type}
      LINK_LIBRARIES
        ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
        ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
        GeneratedWamIkFast SharedLibraryWamIkFast
      SOURCES
        test_IkFast.cpp
    )
  endif()

endif()

dart_format_add(
  GeneratedWamIkFast.cpp
  SharedLibraryWamIkFast.hpp
  SharedLibraryWamIkFast.cpp
)

if(TARGET ${PROJECT_NAME}${DART_MAJOR_VERSION}-io AND NOT MSVC)

  dart_build_tests(
    TYPE ${test_type}
    LINK_LIBRARIES
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
      ${PROJECT_NAME}${DART_MAJOR_VERSION}-test-dynamics
    SOURCES
      test_Collision.cpp
      test_DartLoader.cpp
      test_Dynamics.cpp
      test_ForwardKinematics.cpp
      test_Joints.cpp
      test_JointForceTorque.cpp
      test_MetaSkeleton.cpp
      test_Skeleton.cpp
      test_SoftDynamics.cpp
      test_World.cpp
  )

endif()
