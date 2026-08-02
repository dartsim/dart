# Copyright (c) 2011, The DART development contributors

dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    test_Aspect.cpp
    test_Building.cpp
    test_Common.cpp
    test_Concurrency.cpp
    test_Constraint.cpp
    test_Frames.cpp
    test_Friction.cpp
    test_InvalidDynamicsInputs.cpp
    test_InverseKinematics.cpp
    test_NameManagement.cpp
    test_Signal.cpp
    test_StepAllocation.cpp
)
target_sources(INTEGRATION_StepAllocation PRIVATE AllocationCounting.cpp)
target_link_libraries(INTEGRATION_StepAllocation PRIVATE dart-utils)
if(DART_CODECOV)
  # Coverage instrumentation allocates; keep the allocation counters inert so
  # coverage runs do not report gcov-runtime allocations as DART allocations.
  target_compile_definitions(INTEGRATION_StepAllocation PRIVATE DART_CODECOV=1)
endif()
dart_format_add(AllocationCounting.cpp AllocationCounting.hpp)

dart_add_test("integration" test_CollisionGroups)
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_CollisionGroups ${collision_engine})
  endif()
endforeach()
dart_add_test("integration" test_ConstraintSolver)
target_sources(test_ConstraintSolver PRIVATE AllocationCounting.cpp)
if(DART_CODECOV)
  target_compile_definitions(test_ConstraintSolver PRIVATE DART_CODECOV=1)
endif()
dart_add_test("integration" test_ContactConstraint)
dart_add_test("integration" test_SplitImpulse)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_SplitImpulse dart-collision-bullet)
  target_link_libraries(
    INTEGRATION_StepAllocation
    PRIVATE dart-collision-bullet
  )
endif()
if(TARGET dart-collision-ode)
  target_link_libraries(INTEGRATION_StepAllocation PRIVATE dart-collision-ode)
endif()
dart_add_test("integration" test_IslandDeactivation)
dart_add_test("integration" test_Issue719)
dart_add_test("integration" test_GenericJoints)
dart_add_test("integration" test_Inertia)
dart_add_test("integration" test_Lemke)
dart_add_test("integration" test_LocalResourceRetriever)
dart_add_test("integration" test_Optimizer)
dart_add_test("integration" test_ScrewJoint)
dart_add_test("integration" test_Subscriptions)

set(
  soft_demo_model_test_sources
  test_AdaptiveSoftContactModel.cpp
  test_SoftWormModel.cpp
  test_ContactArrowLayout.cpp
  "${PROJECT_SOURCE_DIR}/examples/demos/scenes/AdaptiveSoftContactModel.cpp"
  "${PROJECT_SOURCE_DIR}/examples/demos/scenes/SoftWormModel.cpp"
  # ContactArrowLayout is the GUI-free half of the host contact visualizer;
  # ContactVisualizer.cpp itself is excluded because it pulls in ImGui.
  "${PROJECT_SOURCE_DIR}/examples/demos/ContactArrowLayout.cpp"
)
target_sources(test_ConstraintSolver PRIVATE ${soft_demo_model_test_sources})
target_include_directories(
  test_ConstraintSolver
  PRIVATE "${PROJECT_SOURCE_DIR}"
)
set_tests_properties(
  test_ConstraintSolver
  PROPERTIES
    ENVIRONMENT "GTEST_FILTER=ConstraintSolver.*"
    FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
)
add_test(
  NAME test_AdaptiveSoftContactModel
  COMMAND
    $<TARGET_FILE:test_ConstraintSolver>
    --gtest_filter=AdaptiveSoftContactModelTest.*
)
add_test(
  NAME test_SoftWormModel
  COMMAND
    $<TARGET_FILE:test_ConstraintSolver> --gtest_filter=SoftWormModelTest.*
)
add_test(
  NAME test_ContactArrowLayout
  COMMAND
    $<TARGET_FILE:test_ConstraintSolver> --gtest_filter=ContactArrowLayoutTest.*
)
set_tests_properties(
  test_AdaptiveSoftContactModel
  test_SoftWormModel
  test_ContactArrowLayout
  PROPERTIES FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
)
dart_format_add(${soft_demo_model_test_sources})

# The soft-foot locomotion model drives the Atlas soft-feet SDF with the
# existing SIMBICON controller, so it needs the URDF and SDF loaders. Those
# components are optional, so the gate only exists when both are built.
if(TARGET dart-utils AND TARGET dart-utils-urdf)
  set(
    soft_foot_simbicon_test_sources
    test_SoftFootSimbiconModel.cpp
    "${PROJECT_SOURCE_DIR}/examples/demos/scenes/soft_foot_simbicon/SoftFootSimbiconModel.cpp"
    # The controller sources are GUI-free and are reused as-is.
    # AtlasSimbiconScene.cpp is deliberately excluded: it is the only file in
    # that directory that pulls in OSG and ImGui.
    "${PROJECT_SOURCE_DIR}/examples/demos/scenes/atlas_simbicon/Controller.cpp"
    "${PROJECT_SOURCE_DIR}/examples/demos/scenes/atlas_simbicon/State.cpp"
    "${PROJECT_SOURCE_DIR}/examples/demos/scenes/atlas_simbicon/StateMachine.cpp"
    "${PROJECT_SOURCE_DIR}/examples/demos/scenes/atlas_simbicon/TerminalCondition.cpp"
  )
  target_sources(
    test_ConstraintSolver
    PRIVATE ${soft_foot_simbicon_test_sources}
  )
  target_link_libraries(test_ConstraintSolver dart-utils dart-utils-urdf)
  # The push-recovery, motor-noise, and noisy-floor sweeps are multi-minute
  # parameter measurements (many settle+observe trials per foot type), so each
  # gets its own CTest entry: bundled with the correctness gates they exceed
  # the 30-minute per-test ceiling on the FreeBSD VM, which runs roughly 35x
  # slower than bare metal.
  add_test(
    NAME test_SoftFootSimbiconModel
    COMMAND
      $<TARGET_FILE:test_ConstraintSolver>
      "--gtest_filter=SoftFootSimbiconModelTest.*:-SoftFootSimbiconModelTest.MeasuresRecoverablePushForBothFeet:SoftFootSimbiconModelTest.MeasuresMotorNoiseToleranceForBothFeet:SoftFootSimbiconModelTest.MeasuresNoisyFloorToleranceForBothFeet"
  )
  add_test(
    NAME test_SoftFootSimbiconPushSweep
    COMMAND
      $<TARGET_FILE:test_ConstraintSolver>
      --gtest_filter=SoftFootSimbiconModelTest.MeasuresRecoverablePushForBothFeet
  )
  add_test(
    NAME test_SoftFootSimbiconMotorNoiseSweep
    COMMAND
      $<TARGET_FILE:test_ConstraintSolver>
      --gtest_filter=SoftFootSimbiconModelTest.MeasuresMotorNoiseToleranceForBothFeet
  )
  add_test(
    NAME test_SoftFootSimbiconNoisyFloorSweep
    COMMAND
      $<TARGET_FILE:test_ConstraintSolver>
      --gtest_filter=SoftFootSimbiconModelTest.MeasuresNoisyFloorToleranceForBothFeet
  )
  set_tests_properties(
    test_SoftFootSimbiconModel
    test_SoftFootSimbiconPushSweep
    test_SoftFootSimbiconMotorNoiseSweep
    test_SoftFootSimbiconNoisyFloorSweep
    PROPERTIES FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
  )
  # Explicit budgets, overriding the CI workflows' `ctest --timeout 1800`
  # default. Measured on the FreeBSD repro VM, this Atlas world steps ~190x
  # slower than bare metal (the correctness gates alone took 1161 s there),
  # so the 30-minute default cannot hold either entry: the gates need ~22
  # minutes and the sweep projects to ~62. Budgets are sized at roughly twice
  # the projection.
  set_tests_properties(test_SoftFootSimbiconModel PROPERTIES TIMEOUT 3600)
  set_tests_properties(test_SoftFootSimbiconPushSweep PROPERTIES TIMEOUT 7200)
  # The two new sweeps run trial counts comparable to the push sweep (their
  # windows are the same 1300 steps; the floor sweep trades fewer levels for a
  # 576-tile broadphase), so they inherit its measured budget.
  set_tests_properties(
    test_SoftFootSimbiconMotorNoiseSweep
    test_SoftFootSimbiconNoisyFloorSweep
    PROPERTIES TIMEOUT 7200
  )
  # The gcov-instrumented coverage job excludes this pair (`--label-exclude
  # no-coverage-value` in pixi's build-coverage): measured there, the gates
  # took 3337 s and the sweep overran even 7200 s at a ~490x slowdown, while
  # contributing no coverage the report can see -- coverage-report strips
  # */examples/* and */tests/* from coverage.info, and the dart-core paths
  # these drive are already executed by test_SoftWormModel and
  # test_AdaptiveSoftContactModel. Every functional CI job still runs both.
  set_tests_properties(
    test_SoftFootSimbiconModel
    test_SoftFootSimbiconPushSweep
    test_SoftFootSimbiconMotorNoiseSweep
    test_SoftFootSimbiconNoisyFloorSweep
    PROPERTIES LABELS "no-coverage-value"
  )
  # The sweeps additionally carry `long-measurement`, excluded wherever the
  # runtime multiplier makes them re-run already-exercised code paths for
  # hours: the FreeBSD VM job (the push sweep passed there in 3674 s but
  # consumed half the job's wall clock), the ASan ctest (the noisy-floor
  # sweep overran 7200 s), and every unoptimized aggregate run
  # (DARTRunCTest.cmake self-excludes outside optimized configurations
  # after the CMAKE_BUILD_TYPE=None lane hit CI's six-hour ceiling). The
  # optimized Release lanes keep them.
  set_tests_properties(
    test_SoftFootSimbiconPushSweep
    test_SoftFootSimbiconMotorNoiseSweep
    test_SoftFootSimbiconNoisyFloorSweep
    PROPERTIES LABELS "no-coverage-value;long-measurement"
  )
  dart_format_add(${soft_foot_simbicon_test_sources})
endif()

if(TARGET dart-utils)
  dart_add_test("integration" test_CompositeResourceRetriever)
  target_link_libraries(test_CompositeResourceRetriever dart-utils)

  dart_add_test("integration" test_DartResourceRetriever)
  target_link_libraries(test_DartResourceRetriever dart-utils)

  dart_add_test("integration" test_FileInfoWorld)
  target_link_libraries(test_FileInfoWorld dart-utils)

  dart_add_test("integration" test_PackageResourceRetriever)
  target_link_libraries(test_PackageResourceRetriever dart-utils)

  dart_add_test("integration" test_MjcfParser)
  target_link_libraries(test_MjcfParser dart-utils)

  dart_add_test("integration" test_SdfParser)
  target_link_libraries(test_SdfParser dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_SdfParser dart-collision-bullet)
  endif()

  dart_add_test("integration" test_MimicConstraint)
  target_link_libraries(test_MimicConstraint dart-utils)

  dart_add_test("integration" test_SkelParser)
  target_link_libraries(test_SkelParser dart-utils)

  if(NOT MSVC)
    dart_add_test("integration" test_VskParser)
    target_link_libraries(test_VskParser dart-utils)
  endif()
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" test_DartLoader)
    target_link_libraries(test_DartLoader dart-utils-urdf)
  endif()

  dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(
    SharedLibraryWamIkFast
    PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
  )
  target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

  dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(
    GeneratedWamIkFast
    PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
  )
  target_compile_options(GeneratedWamIkFast PRIVATE -w)

  if(BUILD_SHARED_LIBS)
    dart_add_test("integration" test_IkFast)
    target_link_libraries(test_IkFast dart-utils-urdf)
    add_dependencies(test_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
  endif()
endif()

dart_format_add(
  GeneratedWamIkFast.cpp
  SharedLibraryWamIkFast.hpp
  SharedLibraryWamIkFast.cpp
)

dart_add_test("integration" test_Distance)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_Distance dart-collision-bullet)
endif()

dart_add_test("integration" test_Raycast)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_Raycast dart-collision-bullet)
endif()

if(TARGET dart-collision-ode)
  # This test doesn't work with FCL because it converts simple shapes to meshes, which makes
  # it difficult to come up with correct test expectations.
  dart_add_test("integration" test_ForceDependentSlip)
  target_link_libraries(test_ForceDependentSlip dart-collision-ode)
endif()

if(TARGET dart-collision-bullet AND TARGET dart-collision-ode)
  dart_add_test("integration" test_CapsuleGroundContact)
  target_link_libraries(
    test_CapsuleGroundContact
    dart-collision-bullet
    dart-collision-ode
  )
endif()

if(TARGET dart-utils)
  dart_add_test("integration" test_Collision)
  target_link_libraries(test_Collision dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_Collision dart-collision-bullet)
  endif()
  if(TARGET dart-collision-ode)
    target_link_libraries(test_Collision dart-collision-ode)
  endif()

  dart_add_test("integration" test_Dynamics)
  target_link_libraries(test_Dynamics dart-utils)

  dart_add_test("integration" test_Joints)
  target_link_libraries(test_Joints dart-utils)

  dart_add_test("integration" test_JointForceTorque)
  target_link_libraries(test_JointForceTorque dart-utils)

  dart_add_test("integration" test_Skeleton)
  target_link_libraries(test_Skeleton dart-utils)

  dart_add_test("integration" test_MetaSkeleton)
  target_link_libraries(test_MetaSkeleton dart-utils)

  dart_add_test("integration" test_SoftDynamics)
  target_link_libraries(test_SoftDynamics dart-utils)

  dart_add_test("integration" test_World)
  target_link_libraries(test_World dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_World dart-collision-bullet)
  endif()

  if(TARGET dart-utils-urdf)
    dart_add_test("integration" test_ForwardKinematics)
    target_link_libraries(test_ForwardKinematics dart-utils-urdf)
  endif()
endif()
