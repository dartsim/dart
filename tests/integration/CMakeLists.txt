# Copyright (c) 2011-2025, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("integration" test_CollisionGroups collision/test_CollisionGroups.cpp)
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_CollisionGroups ${collision_engine})
  endif()
endforeach()

if(TARGET dart-utils)
  dart_add_test("integration" test_Collision collision/test_Collision.cpp)
  target_link_libraries(test_Collision dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_Collision dart-collision-bullet)
  endif()
  if(TARGET dart-collision-ode)
    target_link_libraries(test_Collision dart-collision-ode)
  endif()
endif()

# Additional collision tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" test_self_collision_filtering collision/test_self_collision_filtering.cpp)
  dart_add_test("integration" test_no_false_contacts collision/test_no_false_contacts.cpp)
endif()

if(TARGET dart-collision-bullet)
  dart_add_test("integration" test_collision_accuracy collision/test_collision_accuracy.cpp)
  target_link_libraries(test_collision_accuracy dart-collision-bullet)
endif()

if(TARGET dart-collision-bullet AND TARGET dart-collision-ode)
  dart_add_test("integration" test_plane_shape_collision collision/test_plane_shape_collision.cpp)
  target_link_libraries(test_plane_shape_collision
    dart-collision-bullet
    dart-collision-ode
    dart-utils)

  dart_add_test("integration" test_contact_grouping collision/test_contact_grouping.cpp)
  target_link_libraries(test_contact_grouping
    dart-collision-ode
    dart-utils)
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

dart_add_test("integration" test_ContactConstraint constraint/test_ContactConstraint.cpp)

if(TARGET dart-collision-ode)
  dart_add_test("integration" test_ForceDependentSlip constraint/test_ForceDependentSlip.cpp)
  target_link_libraries(test_ForceDependentSlip dart-collision-ode)
endif()

# Additional constraint tests
if(TARGET dart-utils-urdf)
  dart_add_test("integration" test_lcp_solver_stability constraint/test_lcp_solver_stability.cpp)
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
  dart_add_test("integration" test_Dynamics dynamics/test_Dynamics.cpp)
  target_link_libraries(test_Dynamics dart-utils)

  dart_add_test("integration" test_Joints dynamics/test_Joints.cpp)
  target_link_libraries(test_Joints dart-utils)

  dart_add_test("integration" test_JointForceTorque dynamics/test_JointForceTorque.cpp)
  target_link_libraries(test_JointForceTorque dart-utils)

  dart_add_test("integration" test_Skeleton dynamics/test_Skeleton.cpp)
  target_link_libraries(test_Skeleton dart-utils)

  dart_add_test("integration" test_MetaSkeleton dynamics/test_MetaSkeleton.cpp)
  target_link_libraries(test_MetaSkeleton dart-utils)

  dart_add_test("integration" test_SoftDynamics dynamics/test_SoftDynamics.cpp)
  target_link_libraries(test_SoftDynamics dart-utils)

  # Additional dynamics tests
  dart_add_test("integration" test_servo_joint_limits dynamics/test_servo_joint_limits.cpp)
  target_link_libraries(test_servo_joint_limits dart-utils)

  dart_add_test("integration" test_universal_joint_limits dynamics/test_universal_joint_limits.cpp)
  target_link_libraries(test_universal_joint_limits dart-utils)

  if(TARGET dart-utils-urdf)
    dart_add_test("integration" test_ForwardKinematics dynamics/test_ForwardKinematics.cpp)
    target_link_libraries(test_ForwardKinematics dart-utils-urdf)

    dart_add_test("integration" test_AtlasIK dynamics/test_AtlasIK.cpp)
    target_link_libraries(test_AtlasIK dart-utils-urdf)

    # Additional dynamics tests
    dart_add_test("integration" test_skeleton_state dynamics/test_skeleton_state.cpp)
    dart_add_test("integration" test_skeleton_topology dynamics/test_skeleton_topology.cpp)
  endif()
endif()

# ==============================================================================
# IO Tests
# ==============================================================================
if(TARGET dart-utils)
  dart_add_test("integration" test_MjcfParser io/test_MjcfParser.cpp)
  target_link_libraries(test_MjcfParser dart-utils)

  dart_add_test("integration" test_SdfParser io/test_SdfParser.cpp)
  target_link_libraries(test_SdfParser dart-utils)

  dart_add_test("integration" test_SkelParser io/test_SkelParser.cpp)
  target_link_libraries(test_SkelParser dart-utils)

  if(NOT MSVC)
    dart_add_test("integration" test_VskParser io/test_VskParser.cpp)
    target_link_libraries(test_VskParser dart-utils)
  endif()
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" test_DartLoader io/test_DartLoader.cpp)
    target_link_libraries(test_DartLoader dart-utils-urdf)
  endif()

  # Additional IO tests
  dart_add_test("integration" test_urdf_material_parsing io/test_urdf_material_parsing.cpp)
  target_link_libraries(test_urdf_material_parsing dart-utils-urdf)

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

dart_add_test("integration" test_Optimizer optimization/test_Optimizer.cpp)
if(TARGET dart-optimizer-ipopt)
  target_link_libraries(test_Optimizer dart-optimizer-ipopt)
endif()
if(TARGET dart-optimizer-nlopt)
  target_link_libraries(test_Optimizer dart-optimizer-nlopt)
endif()

if(TARGET dart-optimizer-pagmo)
  dart_add_test("integration" test_MultiObjectiveOptimization optimization/test_MultiObjectiveOptimization.cpp)
  target_link_libraries(test_MultiObjectiveOptimization dart-optimizer-pagmo)
endif()

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
  dart_add_test("integration" test_FileInfoWorld simulation/test_FileInfoWorld.cpp)
  target_link_libraries(test_FileInfoWorld dart-utils)

  dart_add_test("integration" test_World simulation/test_World.cpp)
  target_link_libraries(test_World dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_World dart-collision-bullet)
  endif()

  # Additional simulation tests
  dart_add_test("integration" test_momentum_conservation simulation/test_momentum_conservation.cpp)
  target_link_libraries(test_momentum_conservation dart-utils)
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

dart_add_test("integration" test_LocalResourceRetriever utils/test_LocalResourceRetriever.cpp)
dart_add_test("integration" test_Subscriptions utils/test_Subscriptions.cpp)

if(TARGET dart-utils)
  dart_add_test("integration" test_CompositeResourceRetriever utils/test_CompositeResourceRetriever.cpp)
  target_link_libraries(test_CompositeResourceRetriever dart-utils)

  dart_add_test("integration" test_DartResourceRetriever utils/test_DartResourceRetriever.cpp)
  target_link_libraries(test_DartResourceRetriever dart-utils)

  dart_add_test("integration" test_PackageResourceRetriever utils/test_PackageResourceRetriever.cpp)
  target_link_libraries(test_PackageResourceRetriever dart-utils)
endif()
