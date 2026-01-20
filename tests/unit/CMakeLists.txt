# Copyright (c) 2011, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("unit" UNIT_collision_Distance collision/test_Distance.cpp)

dart_add_test("unit" UNIT_collision_DistanceFilter collision/test_DistanceFilter.cpp)

dart_add_test("unit" UNIT_collision_CollisionGroup collision/test_CollisionGroup.cpp)

dart_add_test("unit" UNIT_collision_Raycast collision/test_Raycast.cpp)

dart_add_test("unit" UNIT_collision_CollisionResult collision/test_CollisionResult.cpp)
dart_add_test(
  "unit"
  UNIT_collision_ConvexMesh
  collision/test_ConvexMeshShapeCollision.cpp)

dart_add_test("unit" UNIT_collision_Contact collision/test_Contact.cpp)

if(DART_HAVE_BULLET)
  dart_add_test("unit" UNIT_collision_BulletContact collision/test_BulletContact.cpp)
endif()

if(DART_HAVE_ODE)
  dart_add_test("unit" UNIT_collision_OdeHeightmap collision/test_OdeHeightmap.cpp)
endif()

dart_add_test(
  "unit" UNIT_collision_FCLCollisionObject collision/test_FCLCollisionObject.cpp)

# ==============================================================================
# Common Tests
# ==============================================================================
set(common_unit_sources
  common/test_CAllocator.cpp
  common/test_Composite.cpp
  common/test_Exception.cpp
  common/test_Factory.cpp
  common/test_FreeListAllocator.cpp
  common/test_Logging.cpp
  common/test_MemoryManager.cpp
  common/test_PoolAllocator.cpp
  common/test_Resource.cpp
  common/test_Result.cpp
  common/test_StlAllocator.cpp
  common/test_Stopwatch.cpp
  common/test_SubjectObserver.cpp
  common/test_String.cpp
  common/test_Uri.cpp
)

if(DART_BUILD_PROFILE AND DART_PROFILE_ENABLE_TEXT)
  list(APPEND common_unit_sources common/test_Profile.cpp)
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
dart_add_test("unit" UNIT_constraint_ConstraintSolver constraint/test_ConstraintSolver.cpp)
dart_add_test(
  "unit" UNIT_constraint_BoxedLcpConstraintSolver constraint/test_BoxedLcpConstraintSolver.cpp)
dart_add_test(
  "unit" UNIT_constraint_JointLimitConstraint constraint/test_JointLimitConstraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_JointConstraint constraint/test_JointConstraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_BalanceConstraint constraint/test_BalanceConstraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_ContactSurface constraint/test_ContactSurface.cpp)

# ==============================================================================
# Dynamics Tests
# ==============================================================================
dart_add_test("unit" UNIT_dynamics_GenericJoints dynamics/test_GenericJoints.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodeExternalForce dynamics/test_BodyNodeExternalForce.cpp)
dart_add_test("unit" UNIT_dynamics_Inertia dynamics/test_Inertia.cpp)
dart_add_test("unit" UNIT_dynamics_ScrewJoint dynamics/test_ScrewJoint.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodePtr dynamics/test_ShapeNodePtr.cpp)
dart_add_test("unit" UNIT_dynamics_MeshShape dynamics/test_MeshShape.cpp)
dart_add_test("unit" UNIT_dynamics_HeightmapShape dynamics/test_HeightmapShape.cpp)
dart_add_test("unit" UNIT_dynamics_SphereShape dynamics/test_SphereShape.cpp)
dart_add_test("unit" UNIT_dynamics_JointCommand dynamics/test_JointCommand.cpp)
dart_add_test("unit" UNIT_dynamics_ConvexMeshShape dynamics/test_ConvexMeshShape.cpp)
dart_add_test("unit" UNIT_dynamics_BodyNodeDerivatives dynamics/test_BodyNodeDerivatives.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodePotentialEnergy dynamics/test_BodyNodePotentialEnergy.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodeInertia dynamics/test_ShapeNodeInertia.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonClone dynamics/test_SkeletonClone.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonAccessors dynamics/test_SkeletonAccessors.cpp)
dart_add_test("unit" UNIT_dynamics_Noexcept dynamics/test_Noexcept.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodeCollisionSignals dynamics/test_BodyNodeCollisionSignals.cpp)
dart_add_test("unit" UNIT_dynamics_VisualAspect dynamics/test_VisualAspect.cpp)
dart_add_test("unit" UNIT_dynamics_WeldJointMerge dynamics/test_WeldJointMerge.cpp)

# Additional dynamics tests
if(TARGET dart-utils-urdf)
  dart_add_test("unit" UNIT_dynamics_CreateShapeNodeApi dynamics/test_CreateShapeNodeApi.cpp)
endif()

# ==============================================================================
# Math Tests
# ==============================================================================
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT
  LINK_LIBRARIES dart
  SOURCES
    math/test_ConfigurationSpace.cpp
    math/test_Geometry.cpp
    math/test_Icosphere.cpp
    math/test_Math.cpp
    math/test_PolygonMesh.cpp
    math/test_Random.cpp
    math/test_ValueEqual.cpp
    math/test_TriMesh.cpp
)

# Convhull test (requires special handling)
if(TARGET dart)
  dart_add_test("unit" UNIT_math_Convhull math/test_Convhull.cpp)
  target_link_libraries(UNIT_math_Convhull dart)
endif()

# Math LCP tests
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT_math_lcp
  LINK_LIBRARIES dart dart-baseline-ode
  SOURCES
    math/lcp/test_DantzigVsODE.cpp
    math/lcp/test_DantzigSolver.cpp
    math/lcp/test_LcpEdgeCases.cpp
    math/lcp/test_LcpComparisonHarness.cpp
    math/lcp/test_LCPTestProblems.cpp
    math/lcp/test_LcpSolversStress.cpp
    math/lcp/test_Lemke.cpp
    math/lcp/test_PGS.cpp
    math/lcp/test_PivotMatrix.cpp
)

# ==============================================================================
# GUI Tests
# ==============================================================================
if(TARGET dart-gui)
  dart_add_test("unit" UNIT_gui_HeightmapShapeNode gui/test_HeightmapShapeNode.cpp)
  dart_add_test(
    "unit" UNIT_gui_ImGuiWindowScaling gui/test_ImGuiWindowScaling.cpp)
  dart_add_test(
    "unit" UNIT_gui_HeadlessViewer gui/test_HeadlessViewer.cpp)
  target_link_libraries(UNIT_gui_HeadlessViewer dart-gui)
  # System ImGui builds can depend on SDL3, which is not available in ASAN CI.
  if(NOT (DART_ENABLE_ASAN AND DART_USE_SYSTEM_IMGUI))
    dart_add_test(
      "unit" UNIT_gui_MeshShapeNodeMaterialUpdates
      gui/test_MeshShapeNodeMaterialUpdates.cpp)
    target_link_libraries(UNIT_gui_MeshShapeNodeMaterialUpdates dart-gui)
    dart_add_test(
      "unit" UNIT_gui_ConvexMeshShapeNode gui/test_ConvexMeshShapeNode.cpp)
    target_link_libraries(UNIT_gui_ConvexMeshShapeNode dart-gui)
    dart_add_test(
      "unit" UNIT_gui_PlaneShapeNode gui/test_PlaneShapeNode.cpp)
    target_link_libraries(UNIT_gui_PlaneShapeNode dart-gui)
  endif()
endif()

# ==============================================================================
# GUI VSG Tests
# ==============================================================================
if(TARGET dart-gui-vsg)
  dart_add_test("unit" UNIT_gui_vsg_GeometryBuilders gui/vsg/test_GeometryBuilders.cpp)
  target_link_libraries(UNIT_gui_vsg_GeometryBuilders dart-gui-vsg dart-collision-experimental)

  dart_add_test("unit" UNIT_gui_vsg_DebugDraw gui/vsg/test_DebugDraw.cpp)
  target_link_libraries(UNIT_gui_vsg_DebugDraw dart-gui-vsg)

  dart_add_test("unit" UNIT_gui_vsg_SimpleViewer gui/vsg/test_SimpleViewer.cpp)
  target_link_libraries(UNIT_gui_vsg_SimpleViewer dart-gui-vsg)

  dart_add_test("unit" UNIT_gui_vsg_CollisionSceneBuilder gui/vsg/test_CollisionSceneBuilder.cpp)
  target_link_libraries(UNIT_gui_vsg_CollisionSceneBuilder dart-gui-vsg dart-collision-experimental)
endif()

# ==============================================================================
# Utils Tests
# ==============================================================================
if(DART_ENABLE_SDFORMAT)
  dart_add_test("unit" test_SdfHelpers utils/test_SdfHelpers.cpp)
  target_link_libraries(test_SdfHelpers dart-utils sdformat::sdformat)
endif()

if(TARGET dart-utils)
  dart_add_test("unit" test_MeshLoader utils/test_MeshLoader.cpp)
  target_link_libraries(test_MeshLoader dart-utils)

  dart_add_test("unit" test_XmlHelpers utils/test_XmlHelpers.cpp)
  target_link_libraries(test_XmlHelpers dart-utils)
endif()

if(TARGET dart-io)
  dart_add_test("unit" test_IoRead io/test_Read.cpp)
  target_link_libraries(test_IoRead dart-io)
endif()

# ==============================================================================
# Simulation-Experimental Tests
# ==============================================================================
if(DART_BUILD_SIMULATION_EXPERIMENTAL)
  add_subdirectory(simulation/experimental)
endif()

# ==============================================================================
# Collision-Experimental Tests
# ==============================================================================
if(DART_BUILD_COLLISION_EXPERIMENTAL)
  add_subdirectory(collision/experimental)
endif()
