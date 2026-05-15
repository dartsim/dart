# Copyright (c) 2011, The DART development contributors

# Reference collision libraries use static Registrar objects to keep legacy
# factory keys available during comparison tests. Tests that do not directly
# reference symbols from these libraries need --no-as-needed to prevent the
# linker from stripping them.
# Force-link a reference collision shared library even if no symbols are
# directly referenced. Without this, --as-needed (default on Linux) strips the
# library and prevents its static Registrar from running.
#
# Usage:
#   dart_force_link_collision_backend(target backend)          # plain mode
#   dart_force_link_collision_backend(target backend PRIVATE)  # keyword mode
function(dart_force_link_collision_backend target backend)
  if(NOT TARGET ${backend})
    return()
  endif()
  set(keyword "${ARGV2}")  # optional: PRIVATE, PUBLIC, etc.
  if(MSVC OR APPLE)
    if(keyword)
      target_link_libraries(${target} ${keyword} ${backend})
    else()
      target_link_libraries(${target} ${backend})
    endif()
  else()
    if(keyword)
      target_link_libraries(
        ${target} ${keyword} -Wl,--no-as-needed ${backend} -Wl,--as-needed)
    else()
      target_link_libraries(
        ${target} -Wl,--no-as-needed ${backend} -Wl,--as-needed)
    endif()
  endif()
endfunction()

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("integration" INTEGRATION_collision_CollisionGroups collision/test_collision_groups.cpp)
dart_add_test("integration" INTEGRATION_collision_Issue2470 collision/test_issue2470.cpp)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  if(TARGET dart-collision-reference-fcl)
    target_link_libraries(INTEGRATION_collision_CollisionGroups dart-collision-reference-fcl)
  endif()
  if(TARGET dart-collision-reference-bullet)
    target_link_libraries(INTEGRATION_collision_CollisionGroups dart-collision-reference-bullet)
  endif()
  if(TARGET dart-collision-reference-ode)
    target_link_libraries(INTEGRATION_collision_CollisionGroups dart-collision-reference-ode)
  endif()
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-io AND TARGET dart-collision-reference-fcl)
  dart_add_test("integration" INTEGRATION_collision_Collision collision/test_collision.cpp)
  target_link_libraries(INTEGRATION_collision_Collision dart-io)
  target_link_libraries(INTEGRATION_collision_Collision dart-collision-reference-fcl)
  if(TARGET dart-collision-reference-bullet)
    target_link_libraries(INTEGRATION_collision_Collision dart-collision-reference-bullet)
  endif()
  if(TARGET dart-collision-reference-ode)
    target_link_libraries(INTEGRATION_collision_Collision dart-collision-reference-ode)
  endif()
  target_link_libraries(INTEGRATION_collision_Collision dart-collision-native)
  target_compile_definitions(INTEGRATION_collision_Collision PRIVATE DART_HAS_DART_COLLISION)
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-collision-reference-fcl)
  dart_add_test(
    "integration"
    INTEGRATION_collision_MeshContactRegression
    collision/test_mesh_contact_regression.cpp)
  target_link_libraries(INTEGRATION_collision_MeshContactRegression dart-collision-reference-fcl)
endif()
if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-collision-reference-fcl)
  dart_add_test(
    "integration"
    INTEGRATION_collision_FclPrimitiveContactMatrix
    collision/test_fcl_primitive_contact_matrix.cpp)
  target_link_libraries(INTEGRATION_collision_FclPrimitiveContactMatrix dart-collision-reference-fcl)
  dart_add_test(
    "integration"
    INTEGRATION_collision_FclPrimitiveContactMatrixFcl
    collision/test_fcl_primitive_contact_matrix_fcl.cpp)
  target_link_libraries(INTEGRATION_collision_FclPrimitiveContactMatrixFcl dart-collision-reference-fcl)
  # Suppress <ciso646> deprecation warning from octomap headers (GCC 15+)
  # This file directly includes <fcl/fcl.h> which pulls in octomap's OcTreeKey.h
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 15)
    target_compile_options(INTEGRATION_collision_FclPrimitiveContactMatrixFcl PRIVATE -Wno-cpp)
  endif()
endif()

dart_add_test(
  "integration"
  INTEGRATION_collision_native_backend_consistency
  collision/test_native_backend_consistency.cpp)
target_link_libraries(
  INTEGRATION_collision_native_backend_consistency
  dart-collision-native)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  target_compile_definitions(
    INTEGRATION_collision_native_backend_consistency
    PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
  )
  if(TARGET dart-collision-reference-fcl)
    target_link_libraries(INTEGRATION_collision_native_backend_consistency dart-collision-reference-fcl)
  endif()
  if(TARGET dart-collision-reference-bullet)
    target_link_libraries(INTEGRATION_collision_native_backend_consistency dart-collision-reference-bullet)
  endif()
  if(TARGET dart-collision-reference-ode)
    target_link_libraries(INTEGRATION_collision_native_backend_consistency dart-collision-reference-ode)
  endif()
endif()

# Additional collision tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" INTEGRATION_collision_SelfCollisionFiltering collision/test_self_collision_filtering.cpp)
  dart_add_test("integration" INTEGRATION_collision_NoFalseContacts collision/test_no_false_contacts.cpp)
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-collision-reference-bullet)
  dart_add_test("integration" INTEGRATION_collision_CollisionAccuracy collision/test_collision_accuracy.cpp)
  target_link_libraries(INTEGRATION_collision_CollisionAccuracy dart-collision-reference-bullet)
  dart_add_test("integration" INTEGRATION_collision_BulletCylinderRolling collision/test_bullet_cylinder_rolling.cpp)
  target_link_libraries(INTEGRATION_collision_BulletCylinderRolling dart-collision-reference-bullet)
  dart_add_test("integration" INTEGRATION_collision_BulletEllipsoidRolling collision/test_bullet_ellipsoid_rolling.cpp)
  target_link_libraries(INTEGRATION_collision_BulletEllipsoidRolling dart-collision-reference-bullet)
  dart_add_test("integration" INTEGRATION_collision_BulletBoxStack collision/test_bullet_box_stack.cpp)
  target_link_libraries(INTEGRATION_collision_BulletBoxStack dart-collision-reference-bullet)
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS
   AND TARGET dart-collision-reference-bullet
   AND TARGET dart-collision-reference-ode
   AND TARGET dart-collision-reference-fcl
)
  dart_add_test("integration" INTEGRATION_collision_PlaneShapeCollision collision/test_plane_shape_collision.cpp)
  target_link_libraries(
    INTEGRATION_collision_PlaneShapeCollision
    dart-utils
    dart-collision-reference-bullet
    dart-collision-reference-ode
    dart-collision-reference-fcl
  )

  if(TARGET dart-io)
    dart_add_test("integration" INTEGRATION_collision_ContactGrouping collision/test_contact_grouping.cpp)
    target_link_libraries(INTEGRATION_collision_ContactGrouping dart-io dart-collision-reference-ode)
  endif()

  dart_add_test(
    "integration"
    INTEGRATION_collision_CapsuleGroundContact
    collision/test_capsule_ground_contact.cpp)
  target_link_libraries(
    INTEGRATION_collision_CapsuleGroundContact
    dart-collision-reference-bullet
    dart-collision-reference-ode
  )

  dart_add_test(
    "integration"
    INTEGRATION_collision_BoxGroundContactParity
    collision/test_box_ground_contact_parity.cpp)
  target_link_libraries(
    INTEGRATION_collision_BoxGroundContactParity
    dart-collision-native
    dart-collision-reference-bullet
    dart-collision-reference-ode
    dart-collision-reference-fcl
  )
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
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  dart_force_link_collision_backend(INTEGRATION_constraint_constraint dart-collision-reference-fcl PRIVATE)
  dart_force_link_collision_backend(INTEGRATION_constraint_friction dart-collision-reference-fcl PRIVATE)
endif()

dart_add_test("integration" INTEGRATION_constraint_ContactConstraint constraint/test_contact_constraint.cpp)
dart_add_test("integration" INTEGRATION_constraint_SplitImpulse constraint/test_split_impulse.cpp)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  dart_force_link_collision_backend(INTEGRATION_constraint_ContactConstraint dart-collision-reference-fcl)
  dart_force_link_collision_backend(INTEGRATION_constraint_SplitImpulse dart-collision-reference-fcl)
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-collision-reference-ode)
  dart_add_test("integration" INTEGRATION_constraint_ForceDependentSlip constraint/test_force_dependent_slip.cpp)
  target_link_libraries(INTEGRATION_constraint_ForceDependentSlip dart-collision-reference-ode)
  dart_force_link_collision_backend(INTEGRATION_constraint_ForceDependentSlip dart-collision-reference-fcl)
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
    dynamics/test_invalid_dynamics_inputs.cpp
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
  if(DART_BUILD_COLLISION_REFERENCE_TESTS)
    dart_force_link_collision_backend(INTEGRATION_dynamics_SoftDynamics dart-collision-reference-fcl)
  endif()

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
    if(DART_BUILD_COLLISION_REFERENCE_TESTS)
      dart_force_link_collision_backend(INTEGRATION_dynamics_SkeletonTopology dart-collision-reference-fcl)
    endif()
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
    if(TARGET dart-io)
      target_link_libraries(INTEGRATION_io_SdfParser dart-io)
    endif()
  endif()

  dart_add_test("integration" INTEGRATION_io_SkelParser io/test_skel_parser.cpp)
  target_link_libraries(INTEGRATION_io_SkelParser dart-utils)
  if(TARGET dart-io)
    target_link_libraries(INTEGRATION_io_SkelParser dart-io)
  endif()
  if(DART_BUILD_COLLISION_REFERENCE_TESTS)
    target_compile_definitions(
      INTEGRATION_io_SkelParser
      PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
    )
    dart_force_link_collision_backend(INTEGRATION_io_SkelParser dart-collision-reference-fcl)
  endif()

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
  if(DART_BUILD_COLLISION_REFERENCE_TESTS)
    target_compile_definitions(
      INTEGRATION_simulation_World
      PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
    )
    if(TARGET dart-collision-reference-fcl)
      target_link_libraries(INTEGRATION_simulation_World dart-collision-reference-fcl)
    endif()
    if(TARGET dart-collision-reference-bullet)
      target_link_libraries(INTEGRATION_simulation_World dart-collision-reference-bullet)
    endif()
    if(TARGET dart-collision-reference-ode)
      target_link_libraries(INTEGRATION_simulation_World dart-collision-reference-ode)
    endif()
  endif()

  # Additional simulation tests
  dart_add_test("integration" INTEGRATION_simulation_Issue410 simulation/test_issue410.cpp)
  target_link_libraries(INTEGRATION_simulation_Issue410 dart-io)
  if(DART_BUILD_COLLISION_REFERENCE_TESTS)
    dart_force_link_collision_backend(INTEGRATION_simulation_Issue410 dart-collision-reference-fcl)
  endif()

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_simulation_MomentumConservation simulation/test_momentum_conservation.cpp)
    target_link_libraries(INTEGRATION_simulation_MomentumConservation dart-io)

    if(DART_BUILD_COLLISION_REFERENCE_TESTS)
      dart_add_test(
        "integration"
        INTEGRATION_simulation_MimicConstraint
        simulation/test_mimic_constraint.cpp
      )
      target_link_libraries(INTEGRATION_simulation_MimicConstraint dart-io)
      target_compile_definitions(
        INTEGRATION_simulation_MimicConstraint
        PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
      )
      dart_force_link_collision_backend(INTEGRATION_simulation_MimicConstraint dart-collision-reference-fcl)
      dart_force_link_collision_backend(INTEGRATION_simulation_MimicConstraint dart-collision-reference-bullet)
      dart_force_link_collision_backend(INTEGRATION_simulation_MimicConstraint dart-collision-reference-ode)
    endif()
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
