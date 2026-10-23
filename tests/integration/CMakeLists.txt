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
)
# The step-allocation scenes load SKEL worlds.
if(TARGET dart-utils)
  dart_build_tests(
    TYPE integration
    TARGET_PREFIX INTEGRATION
    LINK_LIBRARIES dart dart-utils
    SOURCES test_StepAllocation.cpp
  )
  target_sources(INTEGRATION_StepAllocation PRIVATE AllocationCounting.cpp)
  foreach(collision_engine dart-collision-bullet dart-collision-ode)
    if(TARGET ${collision_engine})
      target_link_libraries(
        INTEGRATION_StepAllocation
        PRIVATE ${collision_engine}
      )
    endif()
  endforeach()
  if(DART_CODECOV)
    # Coverage instrumentation allocates; keep the allocation counters inert so
    # coverage runs do not report gcov-runtime allocations as DART allocations.
    target_compile_definitions(
      INTEGRATION_StepAllocation
      PRIVATE DART_CODECOV=1
    )
  endif()
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
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_ConstraintSolver ${collision_engine})
  endif()
endforeach()
if(DART_CODECOV)
  target_compile_definitions(test_ConstraintSolver PRIVATE DART_CODECOV=1)
endif()
dart_add_test("integration" test_ContactConstraint)
dart_add_test("integration" test_GazeboShim)
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_GazeboShim ${collision_engine})
  endif()
endforeach()
dart_add_test("integration" test_SplitImpulse)
if(TARGET dart-collision-ode)
  target_link_libraries(test_SplitImpulse dart-collision-ode)
endif()
if(TARGET dart-collision-bullet)
  target_link_libraries(test_SplitImpulse dart-collision-bullet)
endif()
# Explicit budget for coverage builds, overriding the coverage job's
# `ctest --timeout 3000` default. In the gcov-instrumented Debug coverage
# build this binary measured 1617 s on the green 2026-10-16 nightly but
# overran 3000 s on the slower 2026-10-15 and 2026-10-17 nights (#3797), so
# size at roughly twice the slow-night projection (~3300 s by the suite-wide
# slowdown those nights). Non-coverage runs keep their tighter budgets.
if(DART_CODECOV)
  set_tests_properties(test_SplitImpulse PROPERTIES TIMEOUT 6600)
endif()
# T0 of the friction-solver evaluation. Its scenes are shared with the
# standalone harness in tools/friction_eval, formatted with this tree.
dart_add_test("integration" test_FrictionAnalytic)
target_include_directories(
  test_FrictionAnalytic
  PRIVATE "${PROJECT_SOURCE_DIR}/tools/friction_eval"
)
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_FrictionAnalytic ${collision_engine})
  endif()
endforeach()
# The opt-in NSGS and FBF rows solve to tight tolerances and take tens of
# seconds in Release,
# which the gcov coverage job (roughly 360-600x slower on solver-heavy tests)
# and the FreeBSD VM job (roughly 60-90x) cannot fit in their per-test
# timeouts. They run as a separate entry carrying the labels those jobs
# exclude (see the soft-foot entries below), so the fast Dantzig and PGS rows
# still run everywhere; test_NsgsFrictionSolver and test_FbfFrictionSolver
# still cover the iterative solvers.
set_tests_properties(
  test_FrictionAnalytic
  PROPERTIES
    ENVIRONMENT "GTEST_FILTER=-*/Nsgs*:*/Fbf"
    FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
)
add_test(
  NAME test_FrictionAnalytic_iterative
  COMMAND $<TARGET_FILE:test_FrictionAnalytic> --gtest_filter=*/Nsgs*:*/Fbf
)
set_tests_properties(
  test_FrictionAnalytic_iterative
  PROPERTIES
    FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
    LABELS "no-coverage-value;long-measurement"
)
dart_format_add(
  "${PROJECT_SOURCE_DIR}/tools/friction_eval/friction_eval.cpp"
  "${PROJECT_SOURCE_DIR}/tools/friction_eval/friction_scenes.hpp"
)
dart_add_test("integration" test_IslandDeactivation)
dart_add_test("integration" test_CustomFilterSleeping)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_IslandDeactivation dart-collision-bullet)
  target_link_libraries(test_CustomFilterSleeping dart-collision-bullet)
endif()
if(TARGET dart-collision-ode AND TARGET dart-utils)
  # Covers the `contact_benchmark --gz-preset` helpers.
  dart_add_test("integration" test_ContactBenchmarkGazeboPreset)
  target_link_libraries(
    test_ContactBenchmarkGazeboPreset
    dart-collision-ode
    dart-utils
    ${SDFORMAT_LIBRARIES}
  )
  target_include_directories(
    test_ContactBenchmarkGazeboPreset
    PRIVATE "${PROJECT_SOURCE_DIR}/examples/contact_benchmark"
  )
endif()
dart_add_test("integration" test_Issue719)
dart_add_test("integration" test_GenericJoints)
dart_add_test("integration" test_Inertia)
dart_add_test("integration" test_Lemke)
dart_add_test("integration" test_LocalResourceRetriever)
dart_add_test("integration" test_Optimizer)
dart_add_test("integration" test_ScrewJoint)
dart_add_test("integration" test_Subscriptions)
if(TARGET dart-collision-ode)
  target_link_libraries(test_IslandDeactivation dart-collision-ode)
endif()

set(
  contact_arrow_test_sources
  test_ContactArrowLayout.cpp
  "${PROJECT_SOURCE_DIR}/dart/gui/app/ContactArrowLayout.cpp"
)
target_sources(test_ConstraintSolver PRIVATE ${contact_arrow_test_sources})
set_tests_properties(
  test_ConstraintSolver
  PROPERTIES
    ENVIRONMENT
      "GTEST_FILTER=ConstraintSolver.*:-ConstraintSolver.*BitIdenticalThreadCountResults"
    FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
)
# The NSGS and FBF thread-count determinism checks step 130-box scenes many
# times. Under gcov they outlast test_ConstraintSolver's 3000-second budget
# even at one and two threads (the NSGS check alone took 16 minutes), so the
# coverage job (`--label-exclude no-coverage-value` in
# `pixi run -e coverage build-coverage`)
# skips this entry; every functional CI job still runs it.
add_test(
  NAME test_ConstraintSolver_thread_determinism
  COMMAND
    $<TARGET_FILE:test_ConstraintSolver>
    --gtest_filter=ConstraintSolver.*BitIdenticalThreadCountResults
)
set_tests_properties(
  test_ConstraintSolver_thread_determinism
  PROPERTIES
    FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
    LABELS "no-coverage-value"
)
add_test(
  NAME test_ContactArrowLayout
  COMMAND
    $<TARGET_FILE:test_ConstraintSolver> --gtest_filter=ContactArrowLayoutTest.*
)
set_tests_properties(
  test_ContactArrowLayout
  PROPERTIES FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
)
dart_format_add(
  ${contact_arrow_test_sources}
  "${PROJECT_SOURCE_DIR}/dart/gui/app/ContactArrowLayout.hpp"
)

dart_add_test("integration" test_WorldState)
target_sources(test_WorldState PRIVATE AllocationCounting.cpp)
dart_add_test("integration" test_WorldStateManual)
target_sources(test_WorldStateManual PRIVATE AllocationCounting.cpp)

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
  target_link_libraries(test_SdfParser dart-utils ${SDFORMAT_LIBRARIES})
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

if(TARGET dart-utils-assets)
  find_package(OpenSSL 1.1.1 REQUIRED COMPONENTS Crypto)
  dart_add_test("integration" test_ModelResourceRetriever)
  target_link_libraries(
    test_ModelResourceRetriever
    dart-utils-assets
    OpenSSL::Crypto
  )
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_find_package(urdfdom)
    dart_add_test("integration" test_DartLoader)
    target_link_libraries(test_DartLoader dart-utils-urdf urdfdom)
  endif()
endif()

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

if(TARGET dart-gui-app)
  dart_add_test("integration" test_GuiApp)
  target_link_libraries(test_GuiApp dart-gui-app)
  target_include_directories(
    test_GuiApp
    PRIVATE "${PROJECT_SOURCE_DIR}/examples"
  )
  set_tests_properties(test_GuiApp PROPERTIES LABELS "gui")
else()
  dart_format_add(test_GuiApp.cpp)
endif()

# GUI contracts include actual renderer tests when the optional component is built.
if(TARGET dart-gui)
  dart_add_test("integration" test_GuiAppMemoryDiagnostics)
  dart_add_test("integration" test_GuiAppMemoryDiagnosticsModel)
  foreach(
    memory_test
    test_GuiAppMemoryDiagnostics
    test_GuiAppMemoryDiagnosticsModel
  )
    target_sources(
      ${memory_test}
      PRIVATE
        "${PROJECT_SOURCE_DIR}/dart/gui/app/memory_diagnostics.cpp"
        "${PROJECT_SOURCE_DIR}/dart/gui/app/memory_diagnostics_model.cpp"
    )
    target_link_libraries(${memory_test} dart-gui)
    target_compile_features(${memory_test} PRIVATE cxx_std_17)
    target_compile_definitions(
      ${memory_test}
      PRIVATE
        DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS=1
        DART_DEMOS_HAS_ALLOCATOR_MEMORY_LAYOUT=1
    )
    if(WIN32)
      target_link_libraries(${memory_test} Psapi)
    endif()
    set_tests_properties(
      ${memory_test}
      PROPERTIES LABELS "gui;memory-diagnostics"
    )
  endforeach()
  find_package(PNG REQUIRED)
  find_package(JPEG REQUIRED)
  find_package(glfw3 REQUIRED)
  dart_add_test("integration" test_GuiDescriptors)
  dart_add_test("integration" test_GuiInteractions)
  dart_add_test("integration" test_GuiViewer)
  dart_add_test("integration" test_GuiParity)
  dart_add_test("integration" test_GuiImageDecoders)
  dart_add_test("integration" test_GuiMesh)
  dart_add_test("integration" test_FilamentBackend)
  foreach(
    gui_test
    GuiDescriptors
    GuiInteractions
    GuiViewer
    GuiParity
    GuiImageDecoders
    GuiMesh
    FilamentBackend
  )
    target_link_libraries(test_${gui_test} dart-gui)
    set_tests_properties(test_${gui_test} PROPERTIES LABELS "gui;filament")
    if(gui_test STREQUAL "FilamentBackend")
      target_link_libraries(
        test_${gui_test}
        PNG::PNG
        JPEG::JPEG
        glfw
        Filament::filament
      )
      target_compile_features(test_${gui_test} PRIVATE cxx_std_20)
      set_tests_properties(
        test_${gui_test}
        PROPERTIES LABELS "gui;gui-device;filament;filament-headless"
      )
      set_property(
        TEST test_${gui_test}
        PROPERTY ENVIRONMENT "GTEST_FILTER=FilamentBackend.*"
      )
      add_test(
        NAME test_FilamentNative
        COMMAND test_FilamentBackend --gtest_filter=FilamentNative.*
      )
      set_tests_properties(
        test_FilamentNative
        PROPERTIES LABELS "gui;gui-device;filament-native"
      )
      # A filter that selects nothing must fail instead of passing silently.
      set_tests_properties(
        test_${gui_test}
        test_FilamentNative
        PROPERTIES FAIL_REGULAR_EXPRESSION "(^|[^0-9])0 tests"
      )
    endif()
    if(gui_test STREQUAL "GuiParity" OR gui_test STREQUAL "GuiMesh")
      target_link_libraries(test_${gui_test} PNG::PNG)
      set_tests_properties(
        test_${gui_test}
        PROPERTIES LABELS "gui;gui-device;filament;filament-headless"
      )
    endif()
    if(gui_test STREQUAL "GuiImageDecoders")
      target_link_libraries(test_${gui_test} PNG::PNG JPEG::JPEG)
      target_sources(
        test_${gui_test}
        PRIVATE ${DART_SOURCE_DIR}/dart/gui/detail/image_decoders.cpp
      )
    endif()
    if(NOT gui_test STREQUAL "FilamentBackend")
      target_compile_features(test_${gui_test} PRIVATE cxx_std_17)
    endif()
  endforeach()
else()
  # Lint formats the GUI tests in GUI-off configurations too.
  dart_format_add(
    test_GuiAppMemoryDiagnostics.cpp
    test_GuiAppMemoryDiagnosticsModel.cpp
    test_GuiDescriptors.cpp
    test_GuiInteractions.cpp
    test_GuiViewer.cpp
    test_GuiParity.cpp
    test_GuiImageDecoders.cpp
    test_GuiMesh.cpp
    test_FilamentBackend.cpp
  )
endif()
