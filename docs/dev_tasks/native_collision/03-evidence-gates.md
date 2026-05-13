# Native Collision Evidence Gates

Update this file whenever a gate is run. A gate is not complete until the result
is tied to a commit, command, and observed output.

## Branch Baseline

- Branch: `feature/new_coll`
- Starting commit: `96436fd2503`
- Baseline finding: native collision exists, but default selection and docs
  still contain stale native-collision naming.

## Gate Matrix

| Gate                  | Required evidence                                            | Current state          |
| --------------------- | ------------------------------------------------------------ | ---------------------- |
| Native default        | Focused test showing default detector type is `dart`         | Focused pass, May 2026 |
| Feature parity        | Native unit and integration collision tests pass             | Passed, May 2026       |
| Reference consistency | Native-vs-FCL/Bullet/ODE consistency tests pass              | Focused pass, May 2026 |
| gz-physics            | `pixi run -e gazebo test-gz` passes without patches          | Focused pass, May 2026 |
| Performance           | Comparative benchmarks show native >= best legacy backend    | Focused pass, May 2026 |
| Dependency removal    | Default build succeeds without FCL/Bullet/ODE collision deps | Focused pass, May 2026 |
| Full validation       | `pixi run test-all` passes                                   | Passed, May 2026       |

## Test Runs

- `pixi run lint`
  - Commit: working tree after `96436fd2503`
  - Result: passed. `codespell`, CMake `format`, `clang-format`, `black`,
    `isort`, `prettier`, TOML lint, RST lint, and AI command sync completed.
- `cmake --build build/default/cpp/Release --target
UNIT_constraint_ConstraintSolver UNIT_collision_DartCollisionDetector
UNIT_collision_NativeBackend`
  - Commit: working tree after `96436fd2503`
  - Result: passed. Observed third-party OctoMap `<ciso646>` C++20 warnings
    only.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'UNIT_constraint_ConstraintSolver|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend'`
  - Commit: working tree after `96436fd2503`
  - Result: passed, 3/3 tests.
- `pixi run lint`
  - Commit: working tree after native-default, SKEL fallback, and world default
    edits
  - Result: passed.
- `cmake --build build/default/cpp/Release --target
UNIT_constraint_ConstraintSolver UNIT_collision_DartCollisionDetector
UNIT_collision_NativeBackend UNIT_simulation_World INTEGRATION_simulation_World
INTEGRATION_io_SkelParser`
  - Commit: working tree after native-default, SKEL fallback, and world default
    edits
  - Result: passed. Observed third-party OctoMap `<ciso646>` C++20 warnings
    only.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'UNIT_constraint_ConstraintSolver|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|UNIT_simulation_World|INTEGRATION_simulation_World|INTEGRATION_io_SkelParser'`
  - Commit: working tree after native-default, SKEL fallback, and world default
    edits
  - Result: failed. `UNIT_constraint_ConstraintSolver` exposed a native default
    parity gap: `WorldStepExpandedContactManifold` saw one contact instead of
    the expected multi-contact manifold. The regex also matched unbuilt
    `UNIT_simulation_WorldConfig`; later runs use an anchored regex.
- `cmake --build build/default/cpp/Release --target test_box_box
UNIT_constraint_ConstraintSolver UNIT_simulation_World
INTEGRATION_simulation_World INTEGRATION_io_SkelParser
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend`
  - Commit: working tree after native box-box face patch fix
  - Result: passed.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|UNIT_constraint_ConstraintSolver|UNIT_simulation_World|test_box_box)$'`
  - Commit: working tree after native box-box face patch fix
  - Result: passed, 7/7 tests.
- `pixi run lint`
  - Commit: working tree after docs and native box-box face patch fix
  - Result: passed.
- `cmake --build build/default/cpp/Release --target test_box_box
UNIT_constraint_ConstraintSolver UNIT_simulation_World
INTEGRATION_simulation_World INTEGRATION_io_SkelParser
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend && ctest
--test-dir build/default/cpp/Release --output-on-failure -R
'^(INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|UNIT_constraint_ConstraintSolver|UNIT_simulation_World|test_box_box)$'`
  - Commit: working tree after final lint
  - Result: passed. Build observed only third-party OctoMap `<ciso646>` C++20
    warnings. CTest passed, 7/7 tests.
- `cmake --build build/default/cpp/Release --target
INTEGRATION_collision_native_backend_consistency test_reference_backends &&
ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(INTEGRATION_collision_native_backend_consistency|test_reference_backends)$'`
  - Commit: working tree after final lint
  - Result: passed. Build observed deprecated compatibility-header warnings
    from the reference tests. CTest passed, 2/2 tests.
- `DART_PARALLEL_JOBS=13 CTEST_PARALLEL_LEVEL=13 pixi run -e gazebo test-gz`
  - Commit: working tree after native default and box-box face patch fix
  - Result: failed. DART and gz-physics built successfully, but
    `COMMON_TEST_simulation_features_dartsim` failed
    `UnsupportedRayIntersections`: gz-physics includes the legacy
    `DARTCollisionDetector.hpp` compatibility header and expected the `"dart"`
    detector to report unsupported ray intersections.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector && ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after legacy `DARTCollisionDetector` compatibility
    wrapper
  - Result: passed, 1/1 tests.
- `pixi run -e gazebo cmake --build build/gazebo/cpp --target dart-utils
--parallel 13`
  - Commit: working tree after adding the native cache include to
    `DARTCollisionDetector.hpp`
  - Result: passed. This verified the compatibility header no longer
    instantiates `DartCollisionDetector` with an incomplete native manifold
    cache type in broad include contexts.
- `DART_PARALLEL_JOBS=13 CTEST_PARALLEL_LEVEL=13 pixi run -e gazebo test-gz`
  - Commit: working tree after legacy `DARTCollisionDetector` compatibility
    wrapper and header-completeness fix
  - Result: passed. gz-physics built and CTest passed 65/65 tests. The final
    plugin link check reported a successful DART plugin build with DART
    integration.
- `cmake --build build/default/cpp/Release --target test_box_box
UNIT_constraint_ConstraintSolver UNIT_simulation_World
INTEGRATION_simulation_World INTEGRATION_io_SkelParser
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend
INTEGRATION_collision_native_backend_consistency test_reference_backends
--parallel 13`
  - Commit: working tree after native default, gz-physics compatibility, and
    native performance fixes
  - Result: passed. Build observed only third-party OctoMap `<ciso646>` C++20
    warnings and deprecated compatibility-header warnings in reference tests.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(INTEGRATION_collision_native_backend_consistency|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|UNIT_constraint_ConstraintSolver|UNIT_simulation_World|test_box_box|test_reference_backends)$'`
  - Commit: working tree after native default, gz-physics compatibility, and
    native performance fixes
  - Result: passed, 9/9 tests in 4.99 seconds.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_NativeBackend$'`
  - Commit: working tree after primitive-mesh triangle fast paths
  - Result: passed, 1/1 test.
- `pixi run lint`
  - Commit: working tree after loop-level native collision performance
    optimizations and dense-1000 evidence updates
  - Result: passed. `codespell`, CMake `format`, `clang-format`, `black`,
    `isort`, `prettier`, TOML lint, RST lint, and AI command sync completed.
- `pixi run cmake --build build/default/cpp/Release --target test_box_box
UNIT_constraint_ConstraintSolver UNIT_simulation_World
INTEGRATION_simulation_World INTEGRATION_io_SkelParser
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend
UNIT_collision_CollisionFilter INTEGRATION_collision_native_backend_consistency
test_reference_backends bm_scenarios_mixed_primitives --parallel 13`
  - Commit: working tree after loop-level native collision performance
    optimizations and dense-1000 evidence updates
  - Result: passed. Build observed only third-party OctoMap `<ciso646>` C++20
    warnings and deprecated compatibility-header warnings in reference tests
    and benchmarks.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(INTEGRATION_collision_native_backend_consistency|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_collision_CollisionFilter|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|UNIT_constraint_ConstraintSolver|UNIT_simulation_World|test_box_box|test_reference_backends)$'`
  - Commit: working tree after loop-level native collision performance
    optimizations and dense-1000 evidence updates
  - Result: passed, 10/10 tests in 5.28 seconds.
- `git diff --check`
  - Commit: working tree after loop-level native collision performance
    optimizations and dense-1000 evidence updates
  - Result: passed.
- `pixi run cmake --build build/default/cpp/Release --target
INTEGRATION_collision_Collision INTEGRATION_collision_native_backend_consistency
test_reference_backends test_sdf_compare bm_comparative
bm_comparative_narrow_phase bm_comparative_distance bm_comparative_raycast
bm_scenarios_mixed_primitives bm_scenarios_mesh_heavy
bm_scenarios_raycast_batch bm_scenarios_pipeline_breakdown --parallel 13`
  - Commit: working tree after native naming cleanup
  - Result: passed. Build verified the renamed native benchmark registrations
    and focused collision tests. Observed only existing third-party OctoMap,
    VSG, and deprecated compatibility-header warnings.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(INTEGRATION_collision_Collision|INTEGRATION_collision_native_backend_consistency|test_reference_backends|test_sdf_compare)$'`
  - Commit: working tree after native naming cleanup
  - Result: passed, 4/4 tests.
- Affected benchmark binaries with `--benchmark_list_tests`
  - Commit: working tree after native naming cleanup
  - Result: passed. The affected collision benchmark binaries reported 191
    `Native` benchmark registrations and no stale legacy-named registrations.
- `pixi run lint`
  - Commit: working tree after native naming cleanup
  - Result: passed. `codespell`, CMake `format`, `clang-format`, `black`,
    `isort`, `prettier`, TOML lint, RST lint, and AI command sync completed.
- `pixi run test-all`
  - Commit: working tree before the final all-backends-disabled and
    gz-physics reruns
  - Result: failed full validation. Lint and build completed, and Release
    CTest reached 284/286 passing tests before local GUI failures:
    `UNIT_gui_HeadlessViewer` hung under Mesa/Vulkan headless and
    `UNIT_gui_vsg_geometry_builders` segfaulted during teardown after its
    assertions passed. Full validation remains blocked until these are fixed,
    rerun successfully, or explicitly classified as environment-specific by a
    maintainer.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF pixi run config`
  - Commit: working tree after the `fcl_mesh` fallback, raycast, friction, and
    installed-package compatibility fixes
  - Result: passed. Configure reported `DART_BUILD_COLLISION_FCL: OFF`,
    `DART_BUILD_COLLISION_BULLET: OFF`, and
    `DART_BUILD_COLLISION_ODE: OFF`. FCL was not searched for or listed as a
    required dependency.
- `pixi run cmake --build build/default/cpp/Release --target dart dartpy
dart_collision_native_tests UNIT_collision_CollisionGroup UNIT_collision_Raycast
UNIT_constraint_ContactSurface INTEGRATION_collision_native_backend_consistency
INTEGRATION_io_SkelParser INTEGRATION_simulation_World UNIT_simulation_World
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend
INTEGRATION_constraint_friction INTEGRATION_dynamics_SoftDynamics
INTEGRATION_simulation_Issue410 UNIT_constraint_SoftContactConstraint
--parallel 13`
  - Commit: working tree after all-backends-disabled configure
  - Result: passed. This covered the core library, `dartpy`, native collision
    tests, default detector paths, SKEL loading, contact surface/friction,
    soft-body collision, and simulation regressions with FCL, Bullet, and ODE
    disabled.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_CollisionGroup|UNIT_collision_Raycast|UNIT_constraint_ContactSurface|INTEGRATION_collision_native_backend_consistency|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_simulation_World|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|INTEGRATION_constraint_friction|INTEGRATION_dynamics_SoftDynamics|INTEGRATION_simulation_Issue410|UNIT_constraint_SoftContactConstraint)$'`
  - Commit: working tree after all-backends-disabled focused test build
  - Result: passed, 13/13 tests in 6.92 seconds.
- `pixi run ctest --test-dir build/default/cpp/Release -L collision-native`
  - Commit: working tree after all-backends-disabled native test build
  - Result: passed, 29/29 native collision tests. The label includes AABB
    tree, broad-phase, primitive narrow-phase, convex, mesh, distance, raycast,
    SDF, CCD, filtering, and determinism tests.
- `PYTHONPATH=build/default/cpp/Release/python pixi run pytest -q
python/tests/unit/collision/test_collision.py
python/tests/unit/simulation/test_world.py`
  - Commit: working tree after all-backends-disabled `dartpy` build
  - Result: passed, 9 passed and 1 skipped. The skipped case is the expected
    Bullet detector check when Bullet is disabled.
- `DART_PARALLEL_JOBS=13 CTEST_PARALLEL_LEVEL=13 pixi run -e gazebo test-gz`
  - Commit: working tree after the installed-package `collision-fcl`
    compatibility fix in `DARTConfig.cmake.in`
  - Result: passed. gz-physics found DART with requested components
    `collision-bullet`, `collision-ode`, `utils`, `utils-urdf`, and `dart`, and
    the installed DART package also supplied `collision-fcl` when that optional
    component was installed. This fixed the downstream link regression where
    gz-physics includes `FCLCollisionDetector.hpp` while linking only
    `${DART_LIBRARIES}`. The final plugin link check reported a successful DART
    plugin build with DART integration.
- Native feature audit for `VoxelGridShape`
  - Commit: working tree after the installed-package compatibility fix
  - Result: passed after fixing the final explicit native skip. The native
    DART shape adapter now converts occupied OctoMap leaves from
    `VoxelGridShape` into a native compound of box cells, and the former
    integration-test skip for native voxel grids was removed.
- `pixi run cmake --build build/default/cpp/Release --target
UNIT_collision_NativeBackend INTEGRATION_collision_Collision --parallel 13`
  - Commit: working tree after native `VoxelGridShape` support
  - Result: passed. Build observed only third-party OctoMap `<ciso646>` C++20
    warnings.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_NativeBackend|INTEGRATION_collision_Collision)$'`
  - Commit: working tree after native `VoxelGridShape` support
  - Result: passed, 2/2 tests. This covered the native voxel adapter,
    occupancy-update invalidation, and the collision integration voxel-grid
    scenario.
- `pixi run lint`
  - Commit: working tree after native `VoxelGridShape` support
  - Result: passed. `codespell`, CMake `format`, `clang-format`, `black`,
    `isort`, `prettier`, TOML lint, RST lint, and AI command sync completed.
- `pixi run test-all`
  - Commit: working tree after native `VoxelGridShape` support and GUI
    headless fixes
  - Result: passed. Lint, build, Release C++ tests, simulation-experimental
    tests, Python tests, and documentation all passed. Release CTest reported
    286/286 passing tests, and the script finished with `All tests passed!`
    and `Ready to submit PR!`.
- `DART_PARALLEL_JOBS=13 CTEST_PARALLEL_LEVEL=13 pixi run -e gazebo test-gz`
  - Commit: working tree after native `VoxelGridShape` support and the fresh
    full validation run
  - Result: passed. DART installed in the gazebo environment, gz-physics built,
    gz-physics CTest passed 65/65 tests, and the final plugin link check
    reported a successful DART plugin build with DART integration.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF pixi run config`
  - Commit: working tree after native `VoxelGridShape` support
  - Result: passed. Configure reported `DART_BUILD_COLLISION_FCL: OFF`,
    `DART_BUILD_COLLISION_BULLET: OFF`, and
    `DART_BUILD_COLLISION_ODE: OFF`. FCL was not searched for or listed as a
    required dependency.
- `pixi run cmake --build build/default/cpp/Release --target dart dartpy
dart_collision_native_tests UNIT_collision_CollisionGroup UNIT_collision_Raycast
UNIT_constraint_ContactSurface INTEGRATION_collision_native_backend_consistency
INTEGRATION_io_SkelParser INTEGRATION_simulation_World UNIT_simulation_World
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend
INTEGRATION_constraint_friction INTEGRATION_dynamics_SoftDynamics
INTEGRATION_simulation_Issue410 UNIT_constraint_SoftContactConstraint
--parallel 13`
  - Commit: working tree after all-backends-disabled configure
  - Result: passed. This covered the core library, `dartpy`, native collision
    tests, default detector paths, SKEL loading, contact surface/friction,
    soft-body collision, and simulation regressions with FCL, Bullet, and ODE
    disabled.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_CollisionGroup|UNIT_collision_Raycast|UNIT_constraint_ContactSurface|INTEGRATION_collision_native_backend_consistency|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_simulation_World|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend|INTEGRATION_constraint_friction|INTEGRATION_dynamics_SoftDynamics|INTEGRATION_simulation_Issue410|UNIT_constraint_SoftContactConstraint)$'`
  - Commit: working tree after all-backends-disabled focused test build
  - Result: passed, 13/13 tests.
- `pixi run ctest --test-dir build/default/cpp/Release -L collision-native`
  - Commit: working tree after all-backends-disabled native test build
  - Result: passed, 29/29 native collision tests. The label includes AABB
    tree, broad-phase, primitive narrow-phase, convex, mesh, distance, raycast,
    SDF, CCD, filtering, and determinism tests.
- `PYTHONPATH=build/default/cpp/Release/python pixi run pytest -q
python/tests/unit/collision/test_collision.py
python/tests/unit/simulation/test_world.py`
  - Commit: working tree after all-backends-disabled `dartpy` build
  - Result: passed, 9 passed and 1 skipped. The skipped case is the expected
    Bullet detector check when Bullet is disabled.

## Benchmark Runs

- `cmake --build build/default/cpp/Release --target bm_comparative
bm_comparative_narrow_phase bm_comparative_distance bm_comparative_raycast
bm_scenarios_mixed_primitives bm_scenarios_raycast_batch
bm_scenarios_mesh_heavy`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. All benchmark executables linked under
    `build/default/cpp/Release/bin/`.
- `pixi run bm bm_comparative --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_comparative.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. Accuracy verification passed. Native beat the
    best legacy backend in 7/7 parsed groups.
- `pixi run bm bm_comparative_narrow_phase --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_comparative_narrow_phase.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. Accuracy verification passed. Native beat the
    best legacy backend in 76/77 parsed groups. Remaining loss:
    `BM_NarrowPhase_CylinderBox`: native 1136 ns CPU vs FCL 960 ns CPU.
- `pixi run bm bm_comparative_distance --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_comparative_distance.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. Distance accuracy verification passed. Native
    beat the best legacy backend in 53/66 parsed groups. The 13 losses are
    box-box distance edge cases against Bullet; worst observed case:
    `BM_Distance_EdgeCases/BoxBox_DeepPenetration/1`, native 124 ns CPU vs
    Bullet 79.5 ns CPU.
- `pixi run bm bm_comparative_raycast --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_comparative_raycast.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. Native beat the best legacy backend in 5/5
    parsed raycast groups.
- `pixi run bm bm_scenarios_mixed_primitives --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mixed_primitives.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. Native beat the best legacy backend in 3/6
    parsed groups, but large scenarios fail the performance gate. Worst
    observed losses: dense 10000, native 2.33e10 ns CPU vs FCL 6.98e7 ns CPU;
    sparse 10000, native 4.43e9 ns CPU vs FCL 1.05e8 ns CPU; dense 1000,
    native 7.15e7 ns CPU vs Bullet 3.35e6 ns CPU.
- `pixi run bm bm_scenarios_raycast_batch --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_raycast_batch.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed. Native beat the best legacy backend in 4/4
    parsed batch raycast groups.
- `pixi run bm bm_scenarios_mesh_heavy --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mesh_heavy.json
--benchmark_out_format=json`
  - Commit: working tree after gz-physics compatibility fix
  - Result: passed, but native lost both parsed groups. Mesh-heavy
    1000: native 1.42e8 ns CPU vs Bullet 1.92e7 ns CPU. Mesh-heavy 5000:
    native 7.88e9 ns CPU vs Bullet 7.90e8 ns CPU.
- `pixi run bm bm_comparative_narrow_phase --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_comparative_narrow_phase_after_fastpaths.json
--benchmark_out_format=json`
  - Commit: working tree after box-box, cylinder-box, and scenario
    performance fixes
  - Result: passed. Accuracy verification passed. Native beat the
    best legacy backend in 77/77 parsed groups.
- `pixi run bm bm_comparative_distance --benchmark_filter='(Native|FCL|Bullet)'
--benchmark_min_time=0.2s --benchmark_repetitions=1
--benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_comparative_distance_after_box_fastpath.json
--benchmark_out_format=json`
  - Commit: working tree after box-box distance fast path
  - Result: passed. Distance accuracy verification passed. Native
    beat the best legacy backend in 66/66 parsed groups. ODE was omitted from
    this run because it reports unsupported distance queries and is not a
    meaningful distance-performance reference.
- `pixi run bm bm_scenarios_mixed_primitives --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mixed_primitives_after_count.json
--benchmark_out_format=json`
  - Commit: working tree after cached contact-count tracking and broad-phase
    visitor early exit
  - Result: passed. Native beat the best legacy backend in 5/6
    parsed groups. Remaining loss: dense 1000, native 3.51e6 ns CPU vs Bullet
    2.83e6 ns CPU. The large dense/sparse 10000 cases dropped from
    multi-second O(n^2)-style scans to roughly 2.4e7-2.9e7 ns CPU and now win.
- `pixi run bm bm_scenarios_mixed_primitives --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mixed_primitives_final.json
--benchmark_out_format=json`
  - Commit: working tree after primitive-mesh triangle fast paths
  - Result: passed. Native beat the best legacy backend in 5/6
    parsed groups. Remaining loss: dense 1000, native 3.85e6 ns CPU vs Bullet
    2.63e6 ns CPU.
- `pixi run bm bm_scenarios_mesh_heavy --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mesh_heavy_after_mesh_tuning.json
--benchmark_out_format=json`
  - Commit: working tree after mesh traversal and triangle support-function
    tuning
  - Result: passed. Native beat the best legacy backend in 1/2
    parsed groups. Remaining loss: mesh-heavy 1000, native 3.87e7 ns CPU vs
    Bullet 1.73e7 ns CPU. Mesh-heavy 5000 now wins at roughly 3.68x.
- `pixi run bm bm_scenarios_mesh_heavy --benchmark_min_time=0.2s
--benchmark_repetitions=1 --benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mesh_heavy_after_triangle_fastpaths.json
--benchmark_out_format=json`
  - Commit: working tree after primitive-mesh sphere-triangle and
    capsule-triangle fast paths
  - Result: passed. Native beat the best legacy backend in 2/2
    parsed groups. Mesh-heavy 1000: native 2.90e6 ns CPU vs Bullet 1.95e7 ns
    CPU. Mesh-heavy 5000: native 2.24e7 ns CPU vs Bullet 8.69e8 ns CPU.
- `pixi run bm bm_scenarios_mixed_primitives
--benchmark_filter='BM_Scenario_MixedPrimitives_Dense_(Native|FCL|Bullet|ODE)/1000$'
--benchmark_min_time=0.5s --benchmark_repetitions=3
--benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mixed_primitives_dense1000_after_loop_optimizations.json
--benchmark_out_format=json`
  - Commit: working tree after streaming broad-phase visitation, result reuse,
    filter fast path, and direct primitive-pair dispatch
  - Result: passed, but the performance gate still fails for dense 1000.
    Native improved to 2.81e6 ns CPU mean, but Bullet remains best
    at 1.68e6 ns CPU mean. FCL measured 8.94e6 ns CPU mean and ODE measured
    1.39e8 ns CPU mean. Native reported 70,988 broad-phase pairs, 15,511
    narrow-phase pairs tested, and 10,000 contacts for this case.
- `pixi run bm bm_scenarios_mixed_primitives
--benchmark_filter='BM_Scenario_MixedPrimitives_Dense_(Native|FCL|Bullet|ODE)/1000$'
--benchmark_min_time=0.5s --benchmark_repetitions=3
--benchmark_out=build/default/cpp/Release/benchmark_results/native_collision/bm_scenarios_mixed_primitives_dense1000_after_cached_snapshot_fastpath.json
--benchmark_out_format=json`
  - Commit: working tree after cached broad-phase snapshot reuse in
    `CollisionWorld::collide`
  - Result: passed. Native now beats the best legacy backend for
    the previous remaining mixed-primitives dense 1000 loss: native measured
    1.06e6 ns CPU mean, Bullet measured 1.69e6 ns CPU mean, FCL measured
    9.94e6 ns CPU mean, and ODE measured 1.25e8 ns CPU mean. Native reported
    70,988 broad-phase pairs, 15,511 narrow-phase pairs tested, and 10,000
    contacts for this case. With prior primitive, narrow-phase, supported
    distance, raycast, batch-raycast, mesh-heavy, and mixed-primitive evidence,
    there are no remaining known measured losses in the recorded benchmark set.

## Dependency-Removal Runs

- `DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF pixi run config`
  - Commit: working tree after benchmark runs
  - Result: passed. Configure reported `DART_BUILD_COLLISION_BULLET: OFF` and
    `DART_BUILD_COLLISION_ODE: OFF`, but still found FCL as a required
    dependency.
- `pixi run cmake --build build/default/cpp/Release --target dart --parallel
13`
  - Commit: working tree after Bullet/ODE-disabled configure
  - Result: passed. The core `dart` target built with Bullet and ODE disabled.
    The full dependency-removal gate remains blocked because FCL is still hard
    required by `cmake/dart_find_dependencies.cmake`, `CMakeLists.txt`, and
    pixi dependency metadata.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF pixi run config`
  - Commit: working tree after FCL optionalization
  - Result: passed. Configure reported `DART_BUILD_COLLISION_FCL: OFF`,
    `DART_BUILD_COLLISION_BULLET: OFF`, and `DART_BUILD_COLLISION_ODE: OFF`.
    FCL was not searched for or listed as required.
- `pixi run cmake --build build/default/cpp/Release --target dart --parallel
13`
  - Commit: working tree after all-backends-disabled configure
  - Result: passed. The core `dart` target built without FCL, Bullet, or ODE.
- `pixi run cmake --build build/default/cpp/Release --target
UNIT_collision_CollisionGroup UNIT_collision_Raycast
UNIT_constraint_ContactSurface INTEGRATION_collision_native_backend_consistency
INTEGRATION_io_SkelParser INTEGRATION_simulation_World UNIT_simulation_World
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend --parallel
13`
  - Commit: working tree after all-backends-disabled configure
  - Result: passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_CollisionGroup|UNIT_collision_Raycast|UNIT_constraint_ContactSurface|INTEGRATION_collision_native_backend_consistency|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World|UNIT_simulation_World|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend)$'`
  - Commit: working tree after all-backends-disabled focused test build
  - Result: passed, 9/9 tests.
- `pixi run cmake --build build/default/cpp/Release --target dartpy --parallel
13`
  - Commit: working tree after all-backends-disabled configure
  - Result: passed. This also covered the out-of-line
    `DartCollisionDetector` destructor needed by nanobind when FCL headers are
    absent.
- `PYTHONPATH=build/default/cpp/Release/python pixi run pytest -q
python/tests/unit/collision/test_collision.py
python/tests/unit/simulation/test_world.py`
  - Commit: working tree after all-backends-disabled `dartpy` build
  - Result: passed, 9 passed and 1 skipped.

## Known Risks

- Compatibility-alias checks can silently bypass native-only paths if they are used outside explicit backward-compatibility tests.
- Scenario-scale collision manager performance now passes the recorded
  benchmark set. Dirty-world streaming traversal remains a watch point for
  future simulation-style benchmarks because the dense 1000 win relies on
  reusing an already-valid broad-phase snapshot.
- Native `VoxelGridShape` support adapts occupied OctoMap leaves into compound
  box cells. This closes DART's feature surface and tests, but very large
  voxel grids may need a dedicated acceleration path if they become hot
  simulation workloads.
- gz-physics depends on legacy detector names and compatibility headers; keep
  facade behavior covered while native lowercase APIs evolve.
- Installed-package compatibility remains important: optional legacy component
  files must keep downstream packages such as gz-physics linking when those
  components are built, without making FCL, Bullet, or ODE required for native
  default builds.
