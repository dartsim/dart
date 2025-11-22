# Copyright (c) 2011-2025, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("integration" INTEGRATION_collision_CollisionGroups collision/test_CollisionGroups.cpp)

if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_collision_Collision collision/test_Collision.cpp)
  target_link_libraries(INTEGRATION_collision_Collision dart-utils)
endif()

# Additional collision tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" INTEGRATION_collision_SelfCollisionFiltering collision/test_SelfCollisionFiltering.cpp)
  dart_add_test("integration" INTEGRATION_collision_NoFalseContacts collision/test_NoFalseContacts.cpp)
endif()

if(DART_BUILD_COLLISION_BULLET)
  dart_add_test("integration" INTEGRATION_collision_CollisionAccuracy collision/test_CollisionAccuracy.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletBoxStack collision/test_BulletBoxStack.cpp)
endif()

if(DART_BUILD_COLLISION_BULLET AND DART_BUILD_COLLISION_ODE)
  dart_add_test("integration" INTEGRATION_collision_PlaneShapeCollision collision/test_PlaneShapeCollision.cpp)
  target_link_libraries(INTEGRATION_collision_PlaneShapeCollision dart-utils)

  dart_add_test("integration" INTEGRATION_collision_ContactGrouping collision/test_ContactGrouping.cpp)
  target_link_libraries(INTEGRATION_collision_ContactGrouping dart-utils)

  dart_add_test(
    "integration"
    INTEGRATION_collision_CapsuleGroundContact
    collision/test_CapsuleGroundContact.cpp)
endif()

# ==============================================================================
# Constraint Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    constraint/test_Constraint.cpp
    constraint/test_Friction.cpp
)

dart_add_test("integration" INTEGRATION_constraint_ContactConstraint constraint/test_ContactConstraint.cpp)

if(DART_BUILD_COLLISION_ODE)
  dart_add_test("integration" INTEGRATION_constraint_ForceDependentSlip constraint/test_ForceDependentSlip.cpp)
endif()

# Additional constraint tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" INTEGRATION_constraint_LcpSolverStability constraint/test_LcpSolverStability.cpp)
endif()

# ==============================================================================
# Dynamics Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    dynamics/test_NameManagement.cpp
)

if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_dynamics_Dynamics dynamics/test_Dynamics.cpp)
  target_link_libraries(INTEGRATION_dynamics_Dynamics dart-utils)

  dart_add_test("integration" INTEGRATION_dynamics_Joints dynamics/test_Joints.cpp)
  target_link_libraries(INTEGRATION_dynamics_Joints dart-utils)

  dart_add_test("integration" INTEGRATION_dynamics_JointForceTorque dynamics/test_JointForceTorque.cpp)
  target_link_libraries(INTEGRATION_dynamics_JointForceTorque dart-utils)

  dart_add_test("integration" INTEGRATION_dynamics_Skeleton dynamics/test_Skeleton.cpp)
  target_link_libraries(INTEGRATION_dynamics_Skeleton dart-utils)

  dart_add_test("integration" INTEGRATION_dynamics_MetaSkeleton dynamics/test_MetaSkeleton.cpp)
  target_link_libraries(INTEGRATION_dynamics_MetaSkeleton dart-utils)

  dart_add_test("integration" INTEGRATION_dynamics_SoftDynamics dynamics/test_SoftDynamics.cpp)
  target_link_libraries(INTEGRATION_dynamics_SoftDynamics dart-utils)

  # Additional dynamics tests
  dart_add_test("integration" INTEGRATION_dynamics_ServoJointLimits dynamics/test_ServoJointLimits.cpp)
  target_link_libraries(INTEGRATION_dynamics_ServoJointLimits dart-utils)

  dart_add_test("integration" INTEGRATION_dynamics_UniversalJointLimits dynamics/test_UniversalJointLimits.cpp)
  target_link_libraries(INTEGRATION_dynamics_UniversalJointLimits dart-utils)

  if(TARGET dart-utils-urdf)
    dart_add_test("integration" INTEGRATION_dynamics_ForwardKinematics dynamics/test_ForwardKinematics.cpp)
    target_link_libraries(INTEGRATION_dynamics_ForwardKinematics dart-utils-urdf)

    dart_add_test("integration" INTEGRATION_dynamics_AtlasIK dynamics/test_AtlasIK.cpp)
    target_link_libraries(INTEGRATION_dynamics_AtlasIK dart-utils-urdf)

    # Additional dynamics tests
    dart_add_test("integration" INTEGRATION_dynamics_SkeletonState dynamics/test_SkeletonState.cpp)
    dart_add_test("integration" INTEGRATION_dynamics_SkeletonTopology dynamics/test_SkeletonTopology.cpp)
  endif()
endif()

# ==============================================================================
# IO Tests
# ==============================================================================
if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_io_MjcfParser io/test_MjcfParser.cpp)
  target_link_libraries(INTEGRATION_io_MjcfParser dart-utils)

  dart_add_test("integration" INTEGRATION_io_SdfParser io/test_SdfParser.cpp)
  target_link_libraries(INTEGRATION_io_SdfParser dart-utils)

  dart_add_test("integration" INTEGRATION_io_SkelParser io/test_SkelParser.cpp)
  target_link_libraries(INTEGRATION_io_SkelParser dart-utils)

  if(NOT MSVC)
    dart_add_test("integration" INTEGRATION_io_VskParser io/test_VskParser.cpp)
    target_link_libraries(INTEGRATION_io_VskParser dart-utils)
  endif()
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" INTEGRATION_io_DartLoader io/test_DartLoader.cpp)
    target_link_libraries(INTEGRATION_io_DartLoader dart-utils-urdf)
  endif()

  # Additional IO tests
  dart_add_test("integration" INTEGRATION_io_UrdfMaterialParsing io/test_UrdfMaterialParsing.cpp)
  target_link_libraries(INTEGRATION_io_UrdfMaterialParsing dart-utils-urdf)

  dart_add_library(SharedLibraryWamIkFast io/SharedLibraryWamIkFast.hpp io/SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

  dart_add_library(GeneratedWamIkFast io/GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  target_compile_options(GeneratedWamIkFast PRIVATE -w)

  if(BUILD_SHARED_LIBS)
    dart_add_test("integration" test_IkFast io/test_IkFast.cpp)
    target_link_libraries(test_IkFast dart-utils-urdf)
    add_dependencies(test_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
    target_compile_definitions(
      test_IkFast PRIVATE
        DART_IKFAST_GENERATED_LIB_PATH="$<TARGET_FILE:GeneratedWamIkFast>"
    )
  endif()
endif()

dart_format_add(
  io/GeneratedWamIkFast.cpp
  io/SharedLibraryWamIkFast.hpp
  io/SharedLibraryWamIkFast.cpp
)

# ==============================================================================
# Optimization Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    optimization/test_InverseKinematics.cpp
)

dart_add_test("integration" INTEGRATION_optimization_Optimizer optimization/test_Optimizer.cpp)

# ==============================================================================
# Simulation Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    simulation/test_Building.cpp
)

if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_simulation_FileInfoWorld simulation/test_FileInfoWorld.cpp)
  target_link_libraries(INTEGRATION_simulation_FileInfoWorld dart-utils)

  dart_add_test("integration" INTEGRATION_simulation_World simulation/test_World.cpp)
  target_link_libraries(INTEGRATION_simulation_World dart-utils)

  # Additional simulation tests
  dart_add_test("integration" INTEGRATION_simulation_Issue410 simulation/test_Issue410.cpp)
  target_link_libraries(INTEGRATION_simulation_Issue410 dart-utils)

  dart_add_test("integration" INTEGRATION_simulation_MomentumConservation simulation/test_MomentumConservation.cpp)
  target_link_libraries(INTEGRATION_simulation_MomentumConservation dart-utils)
endif()

# ==============================================================================
# Utils Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    utils/test_Aspect.cpp
    utils/test_Concurrency.cpp
    utils/test_Frames.cpp
    utils/test_Signal.cpp
)

dart_add_test("integration" INTEGRATION_utils_LocalResourceRetriever utils/test_LocalResourceRetriever.cpp)
dart_add_test("integration" INTEGRATION_utils_Subscriptions utils/test_Subscriptions.cpp)

if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_utils_CompositeResourceRetriever utils/test_CompositeResourceRetriever.cpp)
  target_link_libraries(INTEGRATION_utils_CompositeResourceRetriever dart-utils)

  dart_add_test("integration" INTEGRATION_utils_DartResourceRetriever utils/test_DartResourceRetriever.cpp)
  target_link_libraries(INTEGRATION_utils_DartResourceRetriever dart-utils)

  dart_add_test("integration" INTEGRATION_utils_PackageResourceRetriever utils/test_PackageResourceRetriever.cpp)
  target_link_libraries(INTEGRATION_utils_PackageResourceRetriever dart-utils)
endif()
