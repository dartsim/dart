# Copyright (c) 2011-2025, The DART development contributors

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("unit" UNIT_collision_Distance collision/test_Distance.cpp)

dart_add_test("unit" UNIT_collision_DistanceFilter collision/test_DistanceFilter.cpp)

dart_add_test("unit" UNIT_collision_CollisionGroup collision/test_CollisionGroup.cpp)

dart_add_test("unit" UNIT_collision_Raycast collision/test_Raycast.cpp)

dart_add_test("unit" UNIT_collision_CollisionResult collision/test_CollisionResult.cpp)

# ==============================================================================
# Common Tests
# ==============================================================================
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT
  LINK_LIBRARIES dart
  SOURCES
    common/test_CAllocator.cpp
    common/test_Composite.cpp
    common/test_Factory.cpp
    common/test_FreeListAllocator.cpp
    common/test_Logging.cpp
    common/test_MemoryManager.cpp
    common/test_PoolAllocator.cpp
    common/test_Resource.cpp
    common/test_StlAllocator.cpp
    common/test_Stopwatch.cpp
    common/test_SubjectObserver.cpp
    common/test_String.cpp
    common/test_Uri.cpp
)

# ==============================================================================
# Constraint Tests
# ==============================================================================
dart_add_test("unit" UNIT_constraint_ConstraintSolver constraint/test_ConstraintSolver.cpp)
dart_add_test(
  "unit" UNIT_constraint_JointLimitConstraint constraint/test_JointLimitConstraint.cpp)
dart_add_test(
  "unit" UNIT_constraint_BalanceConstraint constraint/test_BalanceConstraint.cpp)

# ==============================================================================
# Dynamics Tests
# ==============================================================================
dart_add_test("unit" UNIT_dynamics_GenericJoints dynamics/test_GenericJoints.cpp)
dart_add_test("unit" UNIT_dynamics_Inertia dynamics/test_Inertia.cpp)
dart_add_test("unit" UNIT_dynamics_ScrewJoint dynamics/test_ScrewJoint.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodePtr dynamics/test_ShapeNodePtr.cpp)
dart_add_test("unit" UNIT_dynamics_MeshShape dynamics/test_MeshShape.cpp)
dart_add_test("unit" UNIT_dynamics_BodyNodeDerivatives dynamics/test_BodyNodeDerivatives.cpp)
dart_add_test(
  "unit" UNIT_dynamics_BodyNodePotentialEnergy dynamics/test_BodyNodePotentialEnergy.cpp)
dart_add_test("unit" UNIT_dynamics_ShapeNodeInertia dynamics/test_ShapeNodeInertia.cpp)
dart_add_test("unit" UNIT_dynamics_SkeletonClone dynamics/test_SkeletonClone.cpp)
dart_add_test("unit" UNIT_dynamics_Noexcept dynamics/test_Noexcept.cpp)

# Additional dynamics tests
if(TARGET dart-utils-urdf)
  dart_add_test("unit" UNIT_dynamics_CreateShapeNodeApi dynamics/test_CreateShapeNodeApi.cpp)
endif()

# ==============================================================================
# LCP Solver Tests (organized in subdirectory)
# ==============================================================================
add_subdirectory(lcpsolver)

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
    math/test_Random.cpp
    math/test_TriMesh.cpp
)

# Convhull test (requires special handling)
if(TARGET dart)
  dart_add_test("unit" UNIT_math_Convhull math/test_Convhull.cpp)
  target_link_libraries(UNIT_math_Convhull dart)
endif()

# ==============================================================================
# Utils Tests
# ==============================================================================
dart_add_test("unit" test_SdfHelpers utils/test_SdfHelpers.cpp)
target_link_libraries(test_SdfHelpers dart-utils sdformat::sdformat)
