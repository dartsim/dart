# Copyright (c) 2011, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("unit" UNIT_collision_Distance collision/test_distance.cpp)

dart_add_test("unit" UNIT_collision_DistanceFilter collision/test_distance_filter.cpp)

dart_add_test("unit" UNIT_collision_CollisionGroup collision/test_collision_group.cpp)

dart_add_test("unit" UNIT_collision_Raycast collision/test_raycast.cpp)

dart_add_test("unit" UNIT_collision_CollisionResult collision/test_collision_result.cpp)
dart_add_test(
  "unit"
  UNIT_collision_ConvexMesh
  collision/test_convex_mesh_shape_collision.cpp)

dart_add_test("unit" UNIT_collision_Contact collision/test_contact.cpp)

if(DART_HAVE_BULLET)
  dart_add_test("unit" UNIT_collision_BulletContact collision/test_bullet_contact.cpp)
endif()

if(DART_HAVE_ODE)
  dart_add_test("unit" UNIT_collision_OdeHeightmap collision/test_ode_heightmap.cpp)
endif()

dart_add_test(
  "unit" UNIT_collision_FCLCollisionObject collision/test_fcl_collision_object.cpp)

# ==============================================================================
# Common Tests
# ==============================================================================
set(common_unit_sources
  common/test_c_allocator.cpp
  common/test_composite.cpp
  common/test_exception.cpp
  common/test_factory.cpp
  common/test_free_list_allocator.cpp
  common/test_logging.cpp
  common/test_memory_manager.cpp
  common/test_pool_allocator.cpp
  common/test_resource.cpp
  common/test_result.cpp
  common/test_stl_allocator.cpp
  common/test_stopwatch.cpp
  common/test_subject_observer.cpp
  common/test_string.cpp
  common/test_uri.cpp
)

if(DART_BUILD_PROFILE AND DART_PROFILE_ENABLE_TEXT)
  list(APPEND common_unit_sources common/test_profile.cpp)
endif()

dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT
  LINK_LIBRARIES dart
  SOURCES
    ${common_unit_sources}
)

# ==============================================================================
# Constraint Tests
# ==============================================================================
dart_add_test("unit" UNIT_constraint_ConstraintSolver constraint/test_constraint_solver.cpp)
dart_add_test(
  "unit" UNIT_constraint_BoxedLcpConstraintSolver constraint/test_boxed_lcp_constraint_solver.cpp)
dart_add_test(
  "unit" UNIT_constraint_JointLimitConstraint constraint/test_joint_limit_constraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_JointConstraint constraint/test_joint_constraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_BalanceConstraint constraint/test_balance_constraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_ContactSurface constraint/test_contact_surface.cpp)

# ==============================================================================
# Dynamics Tests
# ==============================================================================
dart_add_test("unit" UNIT_dynamics_GenericJoints dynamics/test_generic_joints.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodeExternalForce dynamics/test_body_node_external_force.cpp)
dart_add_test("unit" UNIT_dynamics_Inertia dynamics/test_inertia.cpp)
dart_add_test("unit" UNIT_dynamics_ScrewJoint dynamics/test_screw_joint.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodePtr dynamics/test_shape_node_ptr.cpp)
dart_add_test("unit" UNIT_dynamics_MeshShape dynamics/test_mesh_shape.cpp)
dart_add_test("unit" UNIT_dynamics_HeightmapShape dynamics/test_heightmap_shape.cpp)
dart_add_test("unit" UNIT_dynamics_SphereShape dynamics/test_sphere_shape.cpp)
dart_add_test("unit" UNIT_dynamics_JointCommand dynamics/test_joint_command.cpp)
dart_add_test("unit" UNIT_dynamics_ConvexMeshShape dynamics/test_convex_mesh_shape.cpp)
dart_add_test("unit" UNIT_dynamics_BodyNodeDerivatives dynamics/test_body_node_derivatives.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodePotentialEnergy dynamics/test_body_node_potential_energy.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodeInertia dynamics/test_shape_node_inertia.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonClone dynamics/test_skeleton_clone.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonAccessors dynamics/test_skeleton_accessors.cpp)
dart_add_test("unit" UNIT_dynamics_Noexcept dynamics/test_noexcept.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodeCollisionSignals dynamics/test_body_node_collision_signals.cpp)
dart_add_test("unit" UNIT_dynamics_VisualAspect dynamics/test_visual_aspect.cpp)
dart_add_test("unit" UNIT_dynamics_WeldJointMerge dynamics/test_weld_joint_merge.cpp)

# Additional dynamics tests
if(TARGET dart-utils-urdf)
  dart_add_test("unit" UNIT_dynamics_CreateShapeNodeApi dynamics/test_create_shape_node_api.cpp)
endif()

# ==============================================================================
# Math Tests
# ==============================================================================
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT
  LINK_LIBRARIES dart
  SOURCES
    math/test_configuration_space.cpp
    math/test_geometry.cpp
    math/test_icosphere.cpp
    math/test_math.cpp
    math/test_polygon_mesh.cpp
    math/test_random.cpp
    math/test_value_equal.cpp
    math/test_tri_mesh.cpp
)

# Convhull test (requires special handling)
if(TARGET dart)
  dart_add_test("unit" UNIT_math_Convhull math/test_convhull.cpp)
  target_link_libraries(UNIT_math_Convhull dart)
endif()

# Math LCP tests
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT_math_lcp
  LINK_LIBRARIES dart dart-baseline-ode
  SOURCES
    math/lcp/test_dantzig_vs_ode.cpp
    math/lcp/test_dantzig_solver.cpp
    math/lcp/test_lcp_edge_cases.cpp
    math/lcp/test_lcp_comparison_harness.cpp
    math/lcp/test_lcp_test_problems.cpp
    math/lcp/test_lcp_solvers_stress.cpp
    math/lcp/test_lemke.cpp
    math/lcp/test_pgs.cpp
    math/lcp/test_pivot_matrix.cpp
)

# ==============================================================================
# GUI Tests
# ==============================================================================
if(TARGET dart-gui)
  dart_add_test("unit" UNIT_gui_HeightmapShapeNode gui/test_heightmap_shape_node.cpp)
  dart_add_test(
    "unit" UNIT_gui_ImGuiWindowScaling gui/test_im_gui_window_scaling.cpp)
  dart_add_test(
    "unit" UNIT_gui_HeadlessViewer gui/test_headless_viewer.cpp)
  target_link_libraries(UNIT_gui_HeadlessViewer dart-gui)
  # System ImGui builds can depend on SDL3, which is not available in ASAN CI.
  if(NOT (DART_ENABLE_ASAN AND DART_USE_SYSTEM_IMGUI))
    dart_add_test(
      "unit" UNIT_gui_MeshShapeNodeMaterialUpdates
      gui/test_mesh_shape_node_material_updates.cpp)
    target_link_libraries(UNIT_gui_MeshShapeNodeMaterialUpdates dart-gui)
    dart_add_test(
      "unit" UNIT_gui_ConvexMeshShapeNode gui/test_convex_mesh_shape_node.cpp)
    target_link_libraries(UNIT_gui_ConvexMeshShapeNode dart-gui)
    dart_add_test(
      "unit" UNIT_gui_PlaneShapeNode gui/test_plane_shape_node.cpp)
    target_link_libraries(UNIT_gui_PlaneShapeNode dart-gui)
  endif()
endif()

# ==============================================================================
# Utils Tests
# ==============================================================================
if(DART_ENABLE_SDFORMAT)
  dart_add_test("unit" test_sdf_helpersNone utils/test_sdf_helpers.cpp)
  target_link_libraries(test_sdf_helpersNone dart-utils sdformat::sdformat)
endif()

if(TARGET dart-utils)
  dart_add_test("unit" test_mesh_loaderNone utils/test_mesh_loader.cpp)
  target_link_libraries(test_mesh_loaderNone dart-utils)

  dart_add_test("unit" test_xml_helpersNone utils/test_xml_helpers.cpp)
  target_link_libraries(test_xml_helpersNone dart-utils)
endif()

if(TARGET dart-io)
  dart_add_test("unit" test_io_readNone io/test_read.cpp)
  target_link_libraries(test_io_readNone dart-io)
endif()

# ==============================================================================
# Simulation-Experimental Tests
# ==============================================================================
if(DART_BUILD_SIMULATION_EXPERIMENTAL)
  add_subdirectory(simulation/experimental)
endif()

# ==============================================================================
# SIMD Tests
# ==============================================================================
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT_simd
  LINK_LIBRARIES dart-simd GTest::gtest_main
  SOURCES
    simd/test_vec_scalar.cpp
    simd/test_vec_mask_scalar.cpp
    simd/test_operations_scalar.cpp
    simd/test_config.cpp
    simd/test_edge_cases.cpp
)
