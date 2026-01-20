# Copyright (c) 2011, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("integration" INTEGRATION_collision_CollisionGroups collision/test_CollisionGroups.cpp)

if(TARGET dart-io)
  dart_add_test("integration" INTEGRATION_collision_Collision collision/test_Collision.cpp)
  target_link_libraries(INTEGRATION_collision_Collision dart-io)
endif()

if(DART_BUILD_COLLISION_FCL)
  dart_add_test(
    "integration"
    INTEGRATION_collision_MeshContactRegression
    collision/test_MeshContactRegression.cpp)
endif()
dart_add_test(
  "integration"
  INTEGRATION_collision_FclPrimitiveContactMatrix
  collision/test_FclPrimitiveContactMatrix.cpp)
dart_add_test(
  "integration"
  INTEGRATION_collision_FclPrimitiveContactMatrixFcl
  collision/test_FclPrimitiveContactMatrixFcl.cpp)
# Suppress <ciso646> deprecation warning from octomap headers (GCC 15+)
# This file directly includes <fcl/fcl.h> which pulls in octomap's OcTreeKey.h
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 15)
  target_compile_options(INTEGRATION_collision_FclPrimitiveContactMatrixFcl PRIVATE -Wno-cpp)
endif()

if(DART_BUILD_COLLISION_EXPERIMENTAL)
  dart_add_test(
    "integration"
    INTEGRATION_collision_ExperimentalBackendConsistency
    collision/test_ExperimentalBackendConsistency.cpp)
  target_link_libraries(
    INTEGRATION_collision_ExperimentalBackendConsistency
    dart-collision-experimental)
endif()

# Additional collision tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" INTEGRATION_collision_SelfCollisionFiltering collision/test_SelfCollisionFiltering.cpp)
  dart_add_test("integration" INTEGRATION_collision_NoFalseContacts collision/test_NoFalseContacts.cpp)
endif()

if(DART_BUILD_COLLISION_BULLET)
  dart_add_test("integration" INTEGRATION_collision_CollisionAccuracy collision/test_CollisionAccuracy.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletCylinderRolling collision/test_BulletCylinderRolling.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletEllipsoidRolling collision/test_BulletEllipsoidRolling.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletBoxStack collision/test_BulletBoxStack.cpp)
endif()

if(DART_BUILD_COLLISION_BULLET AND DART_BUILD_COLLISION_ODE)
  dart_add_test("integration" INTEGRATION_collision_PlaneShapeCollision collision/test_PlaneShapeCollision.cpp)
  target_link_libraries(INTEGRATION_collision_PlaneShapeCollision dart-utils)

  if(TARGET dart-io)
    dart_add_test("integration" INTEGRATION_collision_ContactGrouping collision/test_ContactGrouping.cpp)
    target_link_libraries(INTEGRATION_collision_ContactGrouping dart-io)
  endif()

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
dart_add_test("integration" INTEGRATION_constraint_SplitImpulse constraint/test_SplitImpulse.cpp)

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

if(TARGET dart-utils AND TARGET dart-io)
  set(DYNAMICS_TEST_FIXTURE dynamics/DynamicsTestFixture.cpp)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsJacobians
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_DynamicsJacobians.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsJacobians dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsFiniteDifference
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_DynamicsFiniteDifference.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsFiniteDifference dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsEquations
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_DynamicsEquations.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsEquations dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsCenterOfMass
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_DynamicsCenterOfMass.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsCenterOfMass dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsImpulse
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_DynamicsImpulse.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsImpulse dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsHybrid
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_DynamicsHybrid.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsHybrid dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_Joints dynamics/test_Joints.cpp)
  target_link_libraries(INTEGRATION_dynamics_Joints dart-io)

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_dynamics_JointForceTorque dynamics/test_JointForceTorque.cpp)
    target_link_libraries(INTEGRATION_dynamics_JointForceTorque dart-io)
  endif()

  dart_add_test("integration" INTEGRATION_dynamics_Skeleton dynamics/test_Skeleton.cpp)
  target_link_libraries(INTEGRATION_dynamics_Skeleton dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_MetaSkeleton dynamics/test_MetaSkeleton.cpp)
  target_link_libraries(INTEGRATION_dynamics_MetaSkeleton dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_SoftDynamics dynamics/test_SoftDynamics.cpp)
  target_link_libraries(INTEGRATION_dynamics_SoftDynamics dart-io)

  # Additional dynamics tests requiring SDF support
  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_dynamics_ServoJointLimits dynamics/test_ServoJointLimits.cpp)
    target_link_libraries(INTEGRATION_dynamics_ServoJointLimits dart-io)

    dart_add_test("integration" INTEGRATION_dynamics_UniversalJointLimits dynamics/test_UniversalJointLimits.cpp)
    target_link_libraries(INTEGRATION_dynamics_UniversalJointLimits dart-io)
  endif()

  if(TARGET dart-utils-urdf)
    dart_add_test("integration" INTEGRATION_dynamics_ForwardKinematics dynamics/test_ForwardKinematics.cpp)
    target_link_libraries(INTEGRATION_dynamics_ForwardKinematics dart-io)

    dart_add_test("integration" INTEGRATION_dynamics_AtlasIK dynamics/test_AtlasIK.cpp)
    target_link_libraries(INTEGRATION_dynamics_AtlasIK dart-io)

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

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_io_SdfParser io/test_SdfParser.cpp)
    target_link_libraries(INTEGRATION_io_SdfParser dart-utils)
  endif()

  dart_add_test("integration" INTEGRATION_io_SkelParser io/test_SkelParser.cpp)
  target_link_libraries(INTEGRATION_io_SkelParser dart-utils)

endif()

if(TARGET dart-io)
  dart_add_test("integration" INTEGRATION_io_Read io/test_Read.cpp)
  target_link_libraries(INTEGRATION_io_Read dart-io)
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" INTEGRATION_io_UrdfParser io/test_UrdfParser.cpp)
    target_link_libraries(INTEGRATION_io_UrdfParser dart-utils-urdf)
  endif()

  # Additional IO tests
  if(TARGET dart-io)
    dart_add_test("integration" INTEGRATION_io_UrdfMaterialParsing io/test_UrdfMaterialParsing.cpp)
    target_link_libraries(INTEGRATION_io_UrdfMaterialParsing dart-io)
  endif()

  dart_add_library(SharedLibraryWamIkFast io/SharedLibraryWamIkFast.hpp io/SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  if(NOT MSVC)
    # Suppress warnings in auto-generated IKFast code without fighting MSVC's /W
    target_compile_options(SharedLibraryWamIkFast PRIVATE -w)
  endif()

  dart_add_library(GeneratedWamIkFast io/GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  if(NOT MSVC)
    target_compile_options(GeneratedWamIkFast PRIVATE -w)
  endif()

  if(BUILD_SHARED_LIBS AND TARGET dart-io)
    dart_add_test("integration" INTEGRATION_io_IkFast io/test_IkFast.cpp)
    target_link_libraries(INTEGRATION_io_IkFast dart-io)
    add_dependencies(INTEGRATION_io_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
    target_compile_definitions(
      INTEGRATION_io_IkFast PRIVATE
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

dart_add_test(
  "integration" INTEGRATION_simulation_Issue870 simulation/test_Issue870.cpp)

dart_add_test(
  "integration" INTEGRATION_simulation_Issue743 simulation/test_Issue743.cpp)

dart_add_test(
  "integration" INTEGRATION_simulation_Sensors simulation/test_Sensors.cpp)

if(TARGET dart-utils AND TARGET dart-io)
  dart_add_test("integration" INTEGRATION_simulation_FileInfoWorld simulation/test_FileInfoWorld.cpp)
  target_link_libraries(INTEGRATION_simulation_FileInfoWorld dart-io)

  dart_add_test("integration" INTEGRATION_simulation_World simulation/test_World.cpp)
  target_link_libraries(INTEGRATION_simulation_World dart-io)

  # Additional simulation tests
  dart_add_test("integration" INTEGRATION_simulation_Issue410 simulation/test_Issue410.cpp)
  target_link_libraries(INTEGRATION_simulation_Issue410 dart-io)

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_simulation_MomentumConservation simulation/test_MomentumConservation.cpp)
    target_link_libraries(INTEGRATION_simulation_MomentumConservation dart-io)

    dart_add_test("integration" INTEGRATION_simulation_MimicConstraint simulation/test_MimicConstraint.cpp)
    target_link_libraries(INTEGRATION_simulation_MimicConstraint dart-io)
  endif()
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
    utils/test_Frames.cpp
    utils/test_Signal.cpp
)

if(DART_ENABLE_SDFORMAT)
  dart_add_test("integration" INTEGRATION_utils_Concurrency utils/test_Concurrency.cpp)
  target_link_libraries(INTEGRATION_utils_Concurrency dart-utils)
endif()

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
