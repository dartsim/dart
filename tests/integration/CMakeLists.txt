# Copyright (c) 2011, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("integration" INTEGRATION_collision_CollisionGroups collision/test_collision_groups.cpp)

if(TARGET dart-io)
  dart_add_test("integration" INTEGRATION_collision_Collision collision/test_collision.cpp)
  target_link_libraries(INTEGRATION_collision_Collision dart-io)
endif()

if(DART_BUILD_COLLISION_FCL)
  dart_add_test(
    "integration"
    INTEGRATION_collision_MeshContactRegression
    collision/test_mesh_contact_regression.cpp)
endif()
dart_add_test(
  "integration"
  INTEGRATION_collision_FclPrimitiveContactMatrix
  collision/test_fcl_primitive_contact_matrix.cpp)
dart_add_test(
  "integration"
  INTEGRATION_collision_FclPrimitiveContactMatrixFcl
  collision/test_fcl_primitive_contact_matrix_fcl.cpp)
# Suppress <ciso646> deprecation warning from octomap headers (GCC 15+)
# This file directly includes <fcl/fcl.h> which pulls in octomap's OcTreeKey.h
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 15)
  target_compile_options(INTEGRATION_collision_FclPrimitiveContactMatrixFcl PRIVATE -Wno-cpp)
endif()

if(DART_BUILD_COLLISION_EXPERIMENTAL)
  dart_add_test(
    "integration"
    INTEGRATION_collision_experimental_backend_consistency
    collision/test_experimental_backend_consistency.cpp)
  target_link_libraries(
    INTEGRATION_collision_experimental_backend_consistency
    dart-collision-experimental)
endif()

# Additional collision tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" INTEGRATION_collision_SelfCollisionFiltering collision/test_self_collision_filtering.cpp)
  dart_add_test("integration" INTEGRATION_collision_NoFalseContacts collision/test_no_false_contacts.cpp)
endif()

if(DART_BUILD_COLLISION_BULLET)
  dart_add_test("integration" INTEGRATION_collision_CollisionAccuracy collision/test_collision_accuracy.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletCylinderRolling collision/test_bullet_cylinder_rolling.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletEllipsoidRolling collision/test_bullet_ellipsoid_rolling.cpp)
  dart_add_test("integration" INTEGRATION_collision_BulletBoxStack collision/test_bullet_box_stack.cpp)
endif()

if(DART_BUILD_COLLISION_BULLET AND DART_BUILD_COLLISION_ODE)
  dart_add_test("integration" INTEGRATION_collision_PlaneShapeCollision collision/test_plane_shape_collision.cpp)
  target_link_libraries(INTEGRATION_collision_PlaneShapeCollision dart-utils)

  if(TARGET dart-io)
    dart_add_test("integration" INTEGRATION_collision_ContactGrouping collision/test_contact_grouping.cpp)
    target_link_libraries(INTEGRATION_collision_ContactGrouping dart-io)
  endif()

  dart_add_test(
    "integration"
    INTEGRATION_collision_CapsuleGroundContact
    collision/test_capsule_ground_contact.cpp)
endif()

# ==============================================================================
# Constraint Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    constraint/test_constraint.cpp
    constraint/test_friction.cpp
)

dart_add_test("integration" INTEGRATION_constraint_ContactConstraint constraint/test_contact_constraint.cpp)
dart_add_test("integration" INTEGRATION_constraint_SplitImpulse constraint/test_split_impulse.cpp)

if(DART_BUILD_COLLISION_ODE)
  dart_add_test("integration" INTEGRATION_constraint_ForceDependentSlip constraint/test_force_dependent_slip.cpp)
endif()

# Additional constraint tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" INTEGRATION_constraint_LcpSolverStability constraint/test_lcp_solver_stability.cpp)
endif()

# ==============================================================================
# Dynamics Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    dynamics/test_name_management.cpp
)

if(TARGET dart-utils AND TARGET dart-io)
  set(DYNAMICS_TEST_FIXTURE dynamics/dynamics_test_fixture.cpp)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsJacobians
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_dynamics_jacobians.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsJacobians dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsFiniteDifference
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_dynamics_finite_difference.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsFiniteDifference dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsEquations
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_dynamics_equations.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsEquations dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsCenterOfMass
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_dynamics_center_of_mass.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsCenterOfMass dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsImpulse
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_dynamics_impulse.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsImpulse dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_DynamicsHybrid
    ${DYNAMICS_TEST_FIXTURE}
    dynamics/test_dynamics_hybrid.cpp)
  target_link_libraries(INTEGRATION_dynamics_DynamicsHybrid dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_Joints dynamics/test_joints.cpp)
  target_link_libraries(INTEGRATION_dynamics_Joints dart-io)

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_dynamics_JointForceTorque dynamics/test_joint_force_torque.cpp)
    target_link_libraries(INTEGRATION_dynamics_JointForceTorque dart-io)
  endif()

  dart_add_test("integration" INTEGRATION_dynamics_Skeleton dynamics/test_skeleton.cpp)
  target_link_libraries(INTEGRATION_dynamics_Skeleton dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_MetaSkeleton dynamics/test_meta_skeleton.cpp)
  target_link_libraries(INTEGRATION_dynamics_MetaSkeleton dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_SoftDynamics dynamics/test_soft_dynamics.cpp)
  target_link_libraries(INTEGRATION_dynamics_SoftDynamics dart-io)

  # Additional dynamics tests requiring SDF support
  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_dynamics_ServoJointLimits dynamics/test_servo_joint_limits.cpp)
    target_link_libraries(INTEGRATION_dynamics_ServoJointLimits dart-io)

    dart_add_test("integration" INTEGRATION_dynamics_UniversalJointLimits dynamics/test_universal_joint_limits.cpp)
    target_link_libraries(INTEGRATION_dynamics_UniversalJointLimits dart-io)
  endif()

  if(TARGET dart-utils-urdf)
    dart_add_test("integration" INTEGRATION_dynamics_ForwardKinematics dynamics/test_forward_kinematics.cpp)
    target_link_libraries(INTEGRATION_dynamics_ForwardKinematics dart-io)

    dart_add_test("integration" INTEGRATION_dynamics_AtlasIK dynamics/test_atlas_ik.cpp)
    target_link_libraries(INTEGRATION_dynamics_AtlasIK dart-io)

    # Additional dynamics tests
    dart_add_test("integration" INTEGRATION_dynamics_SkeletonState dynamics/test_skeleton_state.cpp)
    dart_add_test("integration" INTEGRATION_dynamics_SkeletonTopology dynamics/test_skeleton_topology.cpp)
  endif()
endif()

# ==============================================================================
# IO Tests
# ==============================================================================
if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_io_MjcfParser io/test_mjcf_parser.cpp)
  target_link_libraries(INTEGRATION_io_MjcfParser dart-utils)

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_io_SdfParser io/test_sdf_parser.cpp)
    target_link_libraries(INTEGRATION_io_SdfParser dart-utils)
  endif()

  dart_add_test("integration" INTEGRATION_io_SkelParser io/test_skel_parser.cpp)
  target_link_libraries(INTEGRATION_io_SkelParser dart-utils)

endif()

if(TARGET dart-io)
  dart_add_test("integration" INTEGRATION_io_Read io/test_read.cpp)
  target_link_libraries(INTEGRATION_io_Read dart-io)
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" INTEGRATION_io_UrdfParser io/test_urdf_parser.cpp)
    target_link_libraries(INTEGRATION_io_UrdfParser dart-utils-urdf)
  endif()

  # Additional IO tests
  if(TARGET dart-io)
    dart_add_test("integration" INTEGRATION_io_UrdfMaterialParsing io/test_urdf_material_parsing.cpp)
    target_link_libraries(INTEGRATION_io_UrdfMaterialParsing dart-io)
  endif()

  dart_add_library(SharedLibraryWamIkFast io/shared_library_wam_ik_fast.hpp io/shared_library_wam_ik_fast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(SharedLibraryWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  if(NOT MSVC)
    # Suppress warnings in auto-generated IKFast code without fighting MSVC's /W
    target_compile_options(SharedLibraryWamIkFast PRIVATE -w)
  endif()

  dart_add_library(GeneratedWamIkFast io/generated_wam_ik_fast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(GeneratedWamIkFast PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
  if(NOT MSVC)
    target_compile_options(GeneratedWamIkFast PRIVATE -w)
  endif()

  if(BUILD_SHARED_LIBS AND TARGET dart-io)
    dart_add_test("integration" INTEGRATION_io_IkFast io/test_ik_fast.cpp)
    target_link_libraries(INTEGRATION_io_IkFast dart-io)
    add_dependencies(INTEGRATION_io_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
    target_compile_definitions(
      INTEGRATION_io_IkFast PRIVATE
        DART_IKFAST_GENERATED_LIB_PATH="$<TARGET_FILE:GeneratedWamIkFast>"
    )
  endif()
endif()

dart_format_add(
  io/generated_wam_ik_fast.cpp
  io/shared_library_wam_ik_fast.hpp
  io/shared_library_wam_ik_fast.cpp
)

# ==============================================================================
# Optimization Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    optimization/test_inverse_kinematics.cpp
)

dart_add_test("integration" INTEGRATION_optimization_Optimizer optimization/test_optimizer.cpp)

# ==============================================================================
# Simulation Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    simulation/test_building.cpp
)

dart_add_test(
  "integration" INTEGRATION_simulation_Issue870 simulation/test_issue870.cpp)

dart_add_test(
  "integration" INTEGRATION_simulation_Issue743 simulation/test_issue743.cpp)

dart_add_test(
  "integration" INTEGRATION_simulation_Sensors simulation/test_sensors.cpp)

if(TARGET dart-utils AND TARGET dart-io)
  dart_add_test("integration" INTEGRATION_simulation_FileInfoWorld simulation/test_file_info_world.cpp)
  target_link_libraries(INTEGRATION_simulation_FileInfoWorld dart-io)

  dart_add_test("integration" INTEGRATION_simulation_World simulation/test_world.cpp)
  target_link_libraries(INTEGRATION_simulation_World dart-io)

  # Additional simulation tests
  dart_add_test("integration" INTEGRATION_simulation_Issue410 simulation/test_issue410.cpp)
  target_link_libraries(INTEGRATION_simulation_Issue410 dart-io)

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_simulation_MomentumConservation simulation/test_momentum_conservation.cpp)
    target_link_libraries(INTEGRATION_simulation_MomentumConservation dart-io)

    dart_add_test("integration" INTEGRATION_simulation_MimicConstraint simulation/test_mimic_constraint.cpp)
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
    utils/test_aspect.cpp
    utils/test_frames.cpp
    utils/test_signal.cpp
)

if(DART_ENABLE_SDFORMAT)
  dart_add_test("integration" INTEGRATION_utils_Concurrency utils/test_concurrency.cpp)
  target_link_libraries(INTEGRATION_utils_Concurrency dart-utils)
endif()

dart_add_test("integration" INTEGRATION_utils_LocalResourceRetriever utils/test_local_resource_retriever.cpp)
dart_add_test("integration" INTEGRATION_utils_Subscriptions utils/test_subscriptions.cpp)

if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_utils_CompositeResourceRetriever utils/test_composite_resource_retriever.cpp)
  target_link_libraries(INTEGRATION_utils_CompositeResourceRetriever dart-utils)

  dart_add_test("integration" INTEGRATION_utils_DartResourceRetriever utils/test_dart_resource_retriever.cpp)
  target_link_libraries(INTEGRATION_utils_DartResourceRetriever dart-utils)

  dart_add_test("integration" INTEGRATION_utils_PackageResourceRetriever utils/test_package_resource_retriever.cpp)
  target_link_libraries(INTEGRATION_utils_PackageResourceRetriever dart-utils)
endif()
