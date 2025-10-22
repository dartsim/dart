# Copyright (c) 2011-2025, The DART development contributors

dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    test_Aspect.cpp
    test_Building.cpp
    test_CollisionGroups.cpp
    test_Common.cpp
    test_Concurrency.cpp
    test_Constraint.cpp
    test_ConstraintSolver.cpp
    test_ContactConstraint.cpp
    test_Distance.cpp
    test_Frames.cpp
    test_Friction.cpp
    test_GenericJoints.cpp
    test_Inertia.cpp
    test_InverseKinematics.cpp
    test_Lemke.cpp
    test_LocalResourceRetriever.cpp
    test_NameManagement.cpp
    test_Optimizer.cpp
    test_Raycast.cpp
    test_ScrewJoint.cpp
    test_Signal.cpp
    test_Subscriptions.cpp
)

if(TARGET dart-optimizer-ipopt)
  target_link_libraries(INTEGRATION_Optimizer PRIVATE dart-optimizer-ipopt)
endif()

if(TARGET dart-optimizer-nlopt)
  target_link_libraries(INTEGRATION_Optimizer PRIVATE dart-optimizer-nlopt)
endif()

if(TARGET dart-collision-bullet)
  target_link_libraries(INTEGRATION_CollisionGroups PRIVATE dart-collision-bullet)
  target_link_libraries(INTEGRATION_Distance PRIVATE dart-collision-bullet)
  target_link_libraries(INTEGRATION_Raycast PRIVATE dart-collision-bullet)
endif()

if(TARGET dart-collision-ode)
  target_link_libraries(INTEGRATION_CollisionGroups PRIVATE dart-collision-ode)

  # This test doesn't work with FCL because it converts simple shapes to meshes, which makes
  # it difficult to come up with correct test expectations.
  dart_build_tests(
    TYPE integration
    TARGET_PREFIX INTEGRATION
    LINK_LIBRARIES dart dart-collision-ode
    SOURCES
      test_ForceDependentSlip.cpp
  )
endif()

if(TARGET dart-optimizer-pagmo)
  dart_build_tests(
    TYPE integration
    TARGET_PREFIX INTEGRATION
    LINK_LIBRARIES dart dart-optimizer-pagmo
    SOURCES
      test_MultiObjectiveOptimization.cpp
  )
endif()

if(TARGET dart-utils)
  dart_build_tests(
    TYPE integration
    TARGET_PREFIX INTEGRATION
    LINK_LIBRARIES dart dart-utils
    SOURCES
      test_Collision.cpp
      test_CompositeResourceRetriever.cpp
      test_DartResourceRetriever.cpp
      test_Dynamics.cpp
      test_FileInfoWorld.cpp
      test_JointForceTorque.cpp
      test_Joints.cpp
      test_MetaSkeleton.cpp
      test_MjcfParser.cpp
      test_PackageResourceRetriever.cpp
      test_SdfParser.cpp
      test_SkelParser.cpp
      test_Skeleton.cpp
      test_SoftDynamics.cpp
      test_World.cpp
  )

  if(TARGET dart-collision-bullet)
    target_link_libraries(INTEGRATION_Collision PRIVATE dart-collision-bullet)
    target_link_libraries(INTEGRATION_World PRIVATE dart-collision-bullet)
  endif()

  if(TARGET dart-collision-ode)
    target_link_libraries(INTEGRATION_Collision PRIVATE dart-collision-ode)
  endif()

  if(NOT MSVC)
    dart_build_tests(
      TYPE integration
      TARGET_PREFIX INTEGRATION
      LINK_LIBRARIES dart dart-utils
      SOURCES
        test_VskParser.cpp
    )
  endif()

  if(TARGET dart-utils-urdf)
    dart_build_tests(
      TYPE integration
      TARGET_PREFIX INTEGRATION
      LINK_LIBRARIES dart dart-utils-urdf
      SOURCES
        test_ForwardKinematics.cpp
    )
  endif()
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_build_tests(
      TYPE integration
      TARGET_PREFIX INTEGRATION
      LINK_LIBRARIES dart dart-utils-urdf
      SOURCES
        test_DartLoader.cpp
    )
  endif()

  dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

  dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(GeneratedWamIkFast PRIVATE -w)

  if(BUILD_SHARED_LIBS)
    dart_build_tests(
      TYPE integration
      TARGET_PREFIX INTEGRATION
      LINK_LIBRARIES dart dart-utils-urdf
      SOURCES
        test_IkFast.cpp
    )
    add_dependencies(INTEGRATION_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
  endif()
endif()

dart_format_add(
  GeneratedWamIkFast.cpp
  SharedLibraryWamIkFast.hpp
  SharedLibraryWamIkFast.cpp
)
