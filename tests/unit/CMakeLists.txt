# Copyright (c) 2011, The DART development contributors

# See tests/integration/CMakeLists.txt for documentation on why this is needed.
function(dart_force_link_collision_backend target backend)
  if(NOT TARGET ${backend})
    return()
  endif()
  set(keyword "${ARGV2}")
  if(MSVC OR APPLE)
    if(keyword)
      target_link_libraries(${target} ${keyword} ${backend})
    else()
      target_link_libraries(${target} ${backend})
    endif()
  else()
    if(keyword)
      target_link_libraries(
        ${target}
        ${keyword}
        -Wl,--no-as-needed
        ${backend}
        -Wl,--as-needed
      )
    else()
      target_link_libraries(
        ${target}
        -Wl,--no-as-needed
        ${backend}
        -Wl,--as-needed
      )
    endif()
  endif()
endfunction()

# ==============================================================================
# Collision Tests
# ==============================================================================
if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-test-reference-fcl)
  dart_add_test("unit" UNIT_collision_Distance collision/test_distance.cpp)
  target_link_libraries(UNIT_collision_Distance dart-test-reference-fcl)
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(UNIT_collision_Distance dart-test-reference-bullet)
  endif()
endif()

dart_add_test("unit" UNIT_collision_DistanceFilter collision/test_distance_filter.cpp)

dart_add_test("unit" UNIT_collision_CollisionGroup collision/test_collision_group.cpp)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  target_compile_definitions(
    UNIT_collision_CollisionGroup
    PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
  )
  if(TARGET dart-test-reference-fcl)
    target_link_libraries(UNIT_collision_CollisionGroup dart-test-reference-fcl)
  endif()
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(
      UNIT_collision_CollisionGroup
      dart-test-reference-bullet
    )
  endif()
endif()

dart_add_test(
  "unit" UNIT_collision_DartCollisionDetector collision/test_dart_collision_detector.cpp
)
dart_add_test("unit" UNIT_collision_DARTCollide collision/test_dart_collide.cpp)

dart_add_test("unit" UNIT_collision_Raycast collision/test_raycast.cpp)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  target_compile_definitions(
    UNIT_collision_Raycast
    PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
  )
  if(TARGET dart-test-reference-fcl)
    target_link_libraries(UNIT_collision_Raycast dart-test-reference-fcl)
  endif()
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(UNIT_collision_Raycast dart-test-reference-bullet)
  endif()
endif()

dart_add_test("unit" UNIT_collision_CollisionResult collision/test_collision_result.cpp)
dart_add_test(
  "unit" UNIT_collision_CollisionBackend collision/test_collision_backend.cpp
)
if(
  DART_BUILD_COLLISION_REFERENCE_TESTS
  AND
    (
      TARGET dart-test-reference-fcl
      OR TARGET dart-test-reference-bullet
      OR TARGET dart-test-reference-ode
    )
)
  dart_add_test(
    "unit"
    UNIT_collision_ConvexMesh
    collision/test_convex_mesh_shape_collision.cpp
  )
  if(TARGET dart-test-reference-fcl)
    target_link_libraries(UNIT_collision_ConvexMesh dart-test-reference-fcl)
  endif()
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(UNIT_collision_ConvexMesh dart-test-reference-bullet)
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(UNIT_collision_ConvexMesh dart-test-reference-ode)
  endif()
endif()

dart_add_test("unit" UNIT_collision_Contact collision/test_contact.cpp)

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND DART_HAVE_BULLET)
  dart_add_test("unit" UNIT_collision_BulletContact collision/test_bullet_contact.cpp)
  target_link_libraries(UNIT_collision_BulletContact dart-test-reference-bullet)
  dart_add_test(
    "unit" UNIT_collision_BulletCollisionShapes collision/test_bullet_collision_shapes.cpp
  )
  target_link_libraries(
    UNIT_collision_BulletCollisionShapes
    dart-test-reference-bullet
  )
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND DART_HAVE_ODE)
  dart_add_test("unit" UNIT_collision_OdeHeightmap collision/test_ode_heightmap.cpp)
  target_link_libraries(UNIT_collision_OdeHeightmap dart-test-reference-ode)
  dart_add_test(
    "unit" UNIT_collision_OdeCylinderMesh collision/test_ode_cylinder_mesh.cpp
  )
  target_link_libraries(UNIT_collision_OdeCylinderMesh dart-test-reference-ode)
endif()

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-test-reference-fcl)
  dart_add_test(
    "unit" UNIT_collision_FCLCollisionObject collision/test_fcl_collision_object.cpp
  )
  dart_add_test(
    "unit" UNIT_collision_FCLCollisionDetector collision/test_fcl_collision_detector.cpp
  )
  target_link_libraries(
    UNIT_collision_FCLCollisionObject
    dart-test-reference-fcl
  )
  target_link_libraries(
    UNIT_collision_FCLCollisionDetector
    dart-test-reference-fcl
  )
endif()
dart_add_test("unit" UNIT_collision_CollisionOption collision/test_collision_option.cpp)
dart_add_test("unit" UNIT_collision_CollisionFilter collision/test_collision_filter.cpp)
dart_add_test("unit" UNIT_collision_DistanceOption collision/test_distance_option.cpp)
dart_add_test("unit" UNIT_collision_RaycastOption collision/test_raycast_option.cpp)

# ==============================================================================
# Common Tests
# ==============================================================================
set(
  common_unit_sources
  common/test_aspect.cpp
  common/test_c_allocator.cpp
  common/test_castable.cpp
  common/test_cloneable.cpp
  common/test_composite.cpp
  common/test_exception.cpp
  common/test_factory.cpp
  common/test_feature_support.cpp
  common/test_frame_allocator.cpp
  common/test_free_list_allocator.cpp
  common/test_local_resource.cpp
  common/test_local_resource_retriever.cpp
  common/test_lockable_reference.cpp
  common/test_logging.cpp
  common/test_memory_allocator.cpp
  common/test_memory_manager.cpp
  common/test_name_manager.cpp
  common/test_pool_allocator.cpp
  common/test_profile.cpp
  common/test_profiler.cpp
  common/test_resource.cpp
  common/test_resource_retriever.cpp
  common/test_result.cpp
  common/test_shared_library.cpp
  common/test_signal.cpp
  common/test_singleton.cpp
  common/test_stl_allocator.cpp
  common/test_stopwatch.cpp
  common/test_subject_observer.cpp
  common/test_string.cpp
  common/test_template_coverage.cpp
  common/test_uri.cpp
  common/test_version_counter.cpp
)

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
dart_add_test(
  "unit" UNIT_constraint_BoxedLcpConstraintSolver constraint/test_boxed_lcp_constraint_solver.cpp
)
dart_add_test(
  "unit" UNIT_constraint_BalanceConstraint constraint/test_balance_constraint.cpp
)
dart_add_test(
  "unit" UNIT_constraint_ContactSurface constraint/test_contact_surface.cpp
)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  dart_force_link_collision_backend(UNIT_constraint_ContactSurface dart-test-reference-fcl)
endif()
dart_add_test("unit" UNIT_constraint_JointConstraints constraint/test_joint_constraints.cpp)
dart_add_test(
  "unit"
  UNIT_constraint_CylindricalJointConstraint
  constraint/test_cylindrical_joint_constraint.cpp
)
dart_add_test("unit" UNIT_constraint_ConstrainedGroup constraint/test_constrained_group.cpp)
dart_add_test("unit" UNIT_constraint_NonFiniteContact constraint/test_nonfinite_contact.cpp)

# ==============================================================================
# Dynamics Tests
# ==============================================================================
dart_add_test("unit" UNIT_dynamics_Inertia dynamics/test_inertia.cpp)
dart_add_test("unit" UNIT_dynamics_ScrewJoint dynamics/test_screw_joint.cpp)
dart_add_test("unit" UNIT_dynamics_BallJoint dynamics/test_ball_joint.cpp)
dart_add_test("unit" UNIT_dynamics_UniversalJoint dynamics/test_universal_joint.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodePtr dynamics/test_shape_node_ptr.cpp)
dart_add_test("unit" UNIT_dynamics_BodyNodePtr dynamics/test_body_node_ptr.cpp)
dart_add_test("unit" UNIT_dynamics_NodePtr dynamics/test_node_ptr.cpp)
dart_add_test("unit" UNIT_dynamics_MeshMaterial dynamics/test_mesh_material.cpp)
dart_add_test("unit" UNIT_dynamics_MeshShape dynamics/test_mesh_shape.cpp)
dart_add_test("unit" UNIT_dynamics_HeightmapShape dynamics/test_heightmap_shape.cpp)
dart_add_test("unit" UNIT_dynamics_SphereShape dynamics/test_sphere_shape.cpp)
dart_add_test("unit" UNIT_dynamics_ConvexMeshShape dynamics/test_convex_mesh_shape.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodePotentialEnergy dynamics/test_body_node_potential_energy.cpp
)
dart_add_test("unit" UNIT_dynamics_ShapeNodeInertia dynamics/test_shape_node_inertia.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonClone dynamics/test_skeleton_clone.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonAccessors dynamics/test_skeleton_accessors.cpp)
dart_add_test(
  "unit"
  UNIT_dynamics_ArticulatedDynamicsAlgorithms
  dynamics/test_articulated_dynamics_algorithms.cpp
)
dart_add_test("unit" UNIT_dynamics_BasicNodeManager dynamics/test_basic_node_manager.cpp)
dart_add_test("unit" UNIT_dynamics_Noexcept dynamics/test_noexcept.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodeCollisionSignals dynamics/test_body_node_collision_signals.cpp
)
dart_add_test("unit" UNIT_dynamics_VisualAspect dynamics/test_visual_aspect.cpp)
dart_add_test("unit" UNIT_dynamics_WeldJointMerge dynamics/test_weld_joint_merge.cpp)
dart_add_test("unit" UNIT_dynamics_CapsuleShape dynamics/test_capsule_shape.cpp)
dart_add_test("unit" UNIT_dynamics_ConeShape dynamics/test_cone_shape.cpp)
dart_add_test("unit" UNIT_dynamics_CylinderShape dynamics/test_cylinder_shape.cpp)
dart_add_test("unit" UNIT_dynamics_EllipsoidShape dynamics/test_ellipsoid_shape.cpp)
dart_add_test("unit" UNIT_dynamics_PlaneShape dynamics/test_plane_shape.cpp)
dart_add_test("unit" UNIT_dynamics_PointCloudShape dynamics/test_point_cloud_shape.cpp)
dart_add_test(
  "unit" UNIT_dynamics_SparseOccupancyGrid dynamics/test_sparse_occupancy_grid.cpp
)
if(TARGET octomap)
  target_link_libraries(UNIT_dynamics_SparseOccupancyGrid octomap)
  target_compile_definitions(
    UNIT_dynamics_SparseOccupancyGrid
    PRIVATE DART_TESTS_HAVE_OCTOMAP=1
  )
endif()
dart_add_test("unit" UNIT_dynamics_EndEffector dynamics/test_end_effector.cpp)
dart_add_test("unit" UNIT_dynamics_Marker dynamics/test_marker.cpp)
dart_add_test("unit" UNIT_dynamics_SoftMeshShape dynamics/test_soft_mesh_shape.cpp)
dart_add_test("unit" UNIT_dynamics_Frame dynamics/test_frame.cpp)
dart_add_test("unit" UNIT_dynamics_Group dynamics/test_group.cpp)
dart_add_test("unit" UNIT_dynamics_DegreeOfFreedom dynamics/test_degree_of_freedom.cpp)
dart_add_test("unit" UNIT_dynamics_MetaSkeleton dynamics/test_meta_skeleton.cpp)
dart_add_test("unit" UNIT_dynamics_Linkage dynamics/test_linkage.cpp)
dart_add_test("unit" UNIT_dynamics_LineSegmentShape dynamics/test_line_segment_shape.cpp)
dart_add_test("unit" UNIT_dynamics_PyramidShape dynamics/test_pyramid_shape.cpp)
dart_add_test("unit" UNIT_dynamics_VoxelGridShape dynamics/test_voxel_grid_shape.cpp)
dart_add_test("unit" UNIT_dynamics_TranslationalJoint2D dynamics/test_translational_joint2_d.cpp)
dart_add_test("unit" UNIT_dynamics_ZeroDofJoint dynamics/test_zero_dof_joint.cpp)
dart_add_test("unit" UNIT_dynamics_ReferentialSkeleton dynamics/test_referential_skeleton.cpp)
dart_add_test("unit" UNIT_dynamics_InverseKinematics dynamics/test_inverse_kinematics.cpp)
dart_add_test("unit" UNIT_dynamics_BoxShape dynamics/test_box_shape.cpp)
dart_add_test("unit" UNIT_dynamics_ArrowShape dynamics/test_arrow_shape.cpp)
dart_add_test("unit" UNIT_dynamics_SimpleFrame dynamics/test_simple_frame.cpp)
dart_add_test("unit" UNIT_dynamics_MultiSphereConvexHullShape dynamics/test_multi_sphere_convex_hull_shape.cpp)
dart_add_test("unit" UNIT_dynamics_EulerJoint dynamics/test_euler_joint.cpp)
dart_add_test("unit" UNIT_dynamics_PlanarJoint dynamics/test_planar_joint.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeFrame dynamics/test_shape_frame.cpp)
dart_add_test("unit" UNIT_dynamics_Entity dynamics/test_entity.cpp)

dart_add_test("unit" UNIT_dynamics_BodyNodePool dynamics/test_body_node_pool.cpp)

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

# Optimization tests
dart_add_test("unit" UNIT_math_optimization_Problem math/optimization/test_problem.cpp)
dart_add_test("unit" UNIT_math_optimization_GradientDescentSolver math/optimization/test_gradient_descent_solver.cpp)

# Math LCP tests
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT_math_lcp
  LINK_LIBRARIES dart dart-baseline-ode
  SOURCES
    math/lcp/test_all_solvers_smoke.cpp
    math/lcp/test_dantzig_vs_ode.cpp
    math/lcp/test_dantzig_solver.cpp
    math/lcp/test_dantzig_misc.cpp
    math/lcp/test_lcp_edge_cases.cpp
    math/lcp/test_lcp_comparison_harness.cpp
    math/lcp/test_lcp_generated_coverage.cpp
    math/lcp/test_lcp_test_problems.cpp
    math/lcp/test_lcp_solvers_stress.cpp
    math/lcp/test_lcp_types.cpp
    math/lcp/test_lemke.cpp
    math/lcp/test_pgs.cpp
    math/lcp/test_pivot_matrix.cpp
    math/lcp/test_additional_solvers.cpp
    math/lcp/test_lcp_validation_and_solvers.cpp
    math/lcp/test_lcp_newton_solvers.cpp
    math/lcp/test_lcp_projection_solvers.cpp
)
if(TARGET UNIT_math_lcp_math_lcp_all_solvers_smoke)
  target_compile_definitions(
    UNIT_math_lcp_math_lcp_all_solvers_smoke
    PRIVATE DART_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
  )
endif()
if(TARGET UNIT_math_lcp_math_lcp_lcp_generated_coverage)
  target_compile_definitions(
    UNIT_math_lcp_math_lcp_lcp_generated_coverage
    PRIVATE
      $<$<CONFIG:Debug>:DART_LCP_GENERATED_COVERAGE_SKIP_EXTREME_SINGULAR_DEGENERATE=1>
  )
  if(DART_CODECOV)
    target_compile_definitions(
      UNIT_math_lcp_math_lcp_lcp_generated_coverage
      PRIVATE
        DART_CODECOV=1
        DART_LCP_GENERATED_COVERAGE_SKIP_EXTREME_SINGULAR_DEGENERATE=1
    )
  endif()
endif()

# Math Lie group tests
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT_math_lie_group
  LINK_LIBRARIES dart
  SOURCES
    math/lie_group/test_so3.cpp
    math/lie_group/test_se3.cpp
    math/lie_group/test_group_product.cpp
    math/lie_group/test_spatial_inertia.cpp
    math/lie_group/test_forward_kinematics.cpp
    math/lie_group/test_batch.cpp
)

if(TARGET dart-gui-core)
  dart_add_test("unit" UNIT_gui_DebugVisuals gui/test_gui_debug_visuals.cpp)
  target_link_libraries(UNIT_gui_DebugVisuals dart-gui-core)
endif()

if(TARGET dart-gui AND TARGET dart-imgui-lib)
  # DART-patched ImGui guard tests only work with the fetched ImGui target.
  dart_add_test(
    "unit"
    UNIT_gui_ImGuiColorPickerGuard
    gui/test_imgui_color_picker_guard.cpp
  )
  target_link_libraries(UNIT_gui_ImGuiColorPickerGuard dart-gui dart-imgui-lib)
  dart_add_test(
    "unit" UNIT_gui_ImGuiInternalFormatSymbols
    gui/test_imgui_internal_format_symbols.cpp
  )
  target_link_libraries(
    UNIT_gui_ImGuiInternalFormatSymbols
    dart-imgui-lib
    ${CMAKE_DL_LIBS}
  )
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

  dart_add_test("unit" test_HttpResourceRetriever utils/test_http_resource_retriever.cpp)
  target_link_libraries(test_HttpResourceRetriever dart-utils)

  dart_add_test("unit" test_FileInfoDof utils/test_file_info_dof.cpp)
  target_link_libraries(test_FileInfoDof dart-utils)

  dart_add_test("unit" test_xml_helpersNone utils/test_xml_helpers.cpp)
  target_link_libraries(test_xml_helpersNone dart-utils)
endif()

if(TARGET dart-io)
  dart_add_test("unit" test_io_readNone io/test_read.cpp)
  target_link_libraries(test_io_readNone dart-io)

  dart_add_test("unit" UNIT_io_UrdfParser_Errors io/test_urdf_parser_errors.cpp)
  target_link_libraries(UNIT_io_UrdfParser_Errors dart-io)

  if(DART_ENABLE_SDFORMAT)
    dart_add_test("unit" UNIT_io_SdfParser_Errors io/test_sdf_parser_errors.cpp)
    target_link_libraries(UNIT_io_SdfParser_Errors dart-io)
  endif()
endif()

# ==============================================================================
# Simulation Tests
# ==============================================================================
# Guard on the target, not just the option: the simulation module skips itself in
# minimal configurations where its sibling components (e.g. dart-io) are
# unavailable.
if(TARGET dart-simulation)
  add_subdirectory(simulation)
endif()

# Headless dartsim editor engine tests (links the dartsim_engine library).
if(TARGET dartsim_engine)
  add_subdirectory(dartsim_engine)
endif()

# Testable dartsim UI action/view-model helpers.
if(TARGET dartsim_ui)
  add_subdirectory(dartsim_ui)
endif()

# ==============================================================================
# Collision-Native Tests
# ==============================================================================
add_subdirectory(collision)

# ==============================================================================
# SIMD Tests
# ==============================================================================
# Note: All SIMD tests link Eigen because simd.hpp includes geometry types
# which depend on Eigen for interop support.
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT_simd
  LINK_LIBRARIES dart-simd Eigen3::Eigen GTest::gtest_main
  SOURCES
    simd/test_vec_scalar.cpp
    simd/test_vec_mask_scalar.cpp
    simd/test_operations_scalar.cpp
    simd/test_config.cpp
    simd/test_edge_cases.cpp
    simd/test_eigen_interop.cpp
    simd/test_geometry.cpp
    simd/test_dynamic.cpp
    simd/test_batch_soa.cpp
    simd/test_math.cpp
)
