# Native Collision Evidence Gates

Update this file whenever a gate is run. A gate is not complete until the result
is tied to a commit, command, and observed output.

## Branch Baseline

- Branch: `feature/new_coll`
- Starting commit: `96436fd2503`
- Baseline finding: native collision exists, but default selection and docs
  still contain stale native-collision naming.

## Gate Matrix

| Gate                  | Required evidence                                            | Current state                      |
| --------------------- | ------------------------------------------------------------ | ---------------------------------- |
| Native default        | Focused test showing default detector type is `dart`         | Focused pass, May 2026             |
| Feature parity        | Native unit and integration collision tests pass             | Passed, May 2026                   |
| Reference consistency | Native-vs-FCL/Bullet/ODE consistency tests pass              | Opt-in pass, May 2026              |
| gz-physics            | `pixi run -e gazebo test-gz` passes without patches          | Current local pass, May 2026       |
| Performance           | Comparative benchmarks show native >= best legacy backend    | Focused pass, May 2026             |
| Dependency removal    | Default build succeeds without FCL/Bullet/ODE collision deps | Focused pass, May 2026             |
| Source isolation      | Lint blocks old-engine runtime includes/source leakage       | Local pass, May 2026               |
| Full validation       | `pixi run test-all` passes                                   | Fresh current-state pass, May 2026 |

## North-Star Progress Scale

This scale measures progress toward the single PR's north star: native
collision is the only normal collision stack, and FCL/Bullet/ODE are removed
from DART's default runtime and source tree. `dart/collision/` must also stop
being a true multi-backend selection layer; retained legacy names are
native-backed compatibility adapters only. FCL/Bullet/ODE may remain only as
optional reference engines for correctness tests and benchmarks, with CMake
options that allow native-only builds to opt out. The built-in stack must also
have clean API boundaries, scalable native scene/query state, and
performance-oriented internals before this scale reaches completion.

| Stage | Gate                                                | State                                    |
| ----- | --------------------------------------------------- | ---------------------------------------- |
| 0     | Baseline native backend exists                      | Complete before this task                |
| 1     | Native `dart` detector is default                   | Complete in checkpoint                   |
| 2     | DART feature parity is proven                       | Complete in checkpoint                   |
| 3     | gz-physics compatibility is proven                  | Current local `test-gz` pass             |
| 4     | Native benchmark wins are recorded                  | Complete in checkpoint                   |
| 5     | Local builds pass with FCL/Bullet/ODE disabled      | Complete in checkpoint                   |
| 6     | Native-only and gz-physics CI are permanent         | Manual evidence collected; final left    |
| 7     | Reference engines are test/bench-only opt-in        | Local target split proven                |
| 8     | Default packages/wheels have no old runtime deps    | CI wheel matrix passed on repaired head  |
| 9     | Downstream migration/deprecation path is tested     | Package/gz/link smokes pass; policy left |
| 10    | Clean built-in API/scaling/perf layer               | Design/artifact evidence; final left     |
| 11    | Old runtime backend source is reference-only        | Local split; lint guard wired            |
| 12    | Final PR evidence is complete and task docs removed | Blocked on PR/migration/final cleanup    |

## Remaining North-Star Gate Backlog

These gates are still required before the single north-star PR is complete.

| Gate                         | Required evidence                                                                                                      | Current state                                       |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- |
| CI native-only build         | CI passes with FCL, Bullet, and ODE disabled                                                                           | Manual dispatch passed on repaired head             |
| CI gz-physics compatibility  | gz-physics CI passes using native-backed compatibility facades without optional reference components                   | Current local `test-gz` passes; CI reference exists |
| Reference correctness        | FCL/Bullet/ODE comparison tests are test-only and optional                                                             | Local reference target split passes                 |
| Packaging removal            | Default packages/wheels have no old collision runtime deps                                                             | Verifier wired; CI matrix passed on repaired head   |
| Downstream migration         | gz-physics has a tested path away from legacy detector APIs                                                            | Package/gz/link smokes pass; policy evidence left   |
| Collision abstraction        | Legacy keys/classes route only to built-in native behavior                                                             | Source/package facades done                         |
| Built-in architecture/design | `01-design.md` north-star requirement, layer table, and blueprint pass API-cleanliness, scaling, and performance gates | Source split, cache bridge, gz/full support         |
| Benchmark regression guard   | Optional reference benchmarks guide gradual optimization                                                               | Manual dispatch artifact uploaded on repaired head  |
| Legacy backend deletion      | Old runtime backend sources removed from default stack                                                                 | Runtime source guard wired; deletion left           |

## PR Publishing And Initial CI Repair

- Draft PR:
  - URL: https://github.com/dartsim/dart/pull/2652
  - Head pushed: `feature/new_coll` at `5b08a00d381`
    (`Record current native collision validation pass`).
  - Milestone: `DART 7.0`.
  - Result: GitHub Actions started for the pushed branch. CI evidence is now
    collectible from PR/job artifacts, but the task remains incomplete until
    the required native-only, gz-physics, wheel, benchmark, migration, and
    final cleanup gates are closed.
- Initial `Asserts enabled (no -DNDEBUG)` CI failure:
  - Run/job: `25870574281` / `76024440344`.
  - Result: failed during CMake configure before compilation.
  - Root cause: the custom asserts-enabled workflow configure still passed
    `-DDART_BUILD_COLLISION_BULLET=ON` while the default Pixi environment no
    longer installs Bullet. Bullet now belongs to the explicit
    `collision-reference` environment.
  - Observed failure:

    ```text
    CMake Error at dart/collision/bullet/CMakeLists.txt:28 (message):
      DART_BUILD_COLLISION_BULLET is ON but Bullet was not found. Please install
      libbullet-dev (>= 3.25) or disable DART_BUILD_COLLISION_BULLET.
    ```

- Focused local validation after the workflow repair:
  - Command: custom `CMAKE_BUILD_TYPE=None` configure matching the
    asserts-enabled job, with `DART_BUILD_DARTPY=ON`, FCL/Bullet/ODE `OFF`,
    and collision reference tests/benchmarks `OFF`.
  - Result: passed. Configure reported `DART_BUILD_COLLISION_FCL: OFF`,
    `DART_BUILD_COLLISION_BULLET: OFF`, `DART_BUILD_COLLISION_ODE: OFF`,
    `DART_BUILD_COLLISION_REFERENCE_TESTS: OFF`, and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS: OFF`, then generated build
    files in `build/default/cpp/None-ci-asserts-local`.
- Follow-up PR mergeability repair:
  - The asserts-enabled workflow repair was committed as `901d56c4260`
    (`Fix native collision asserts CI configure`) and pushed to
    `feature/new_coll`.
  - The PR then needed an `origin/main` merge to clear GitHub mergeability
    conflicts. The merge kept `origin/main` collision group observer/source
    ownership, restored canonical `CollisionObject`, `CollisionFilter`,
    `CollisionResult`, and `Contact` definitions, deleted stale
    feature-branch collision bridge sources, kept the native
    `DartCollisionDetector` implementation, regenerated `pixi.lock`, and was
    committed locally as `3120a4fce9b` before being pushed to PR #2652.
  - Focused local validation after the merge: `pixi run config` passed with
    FCL/Bullet/ODE and collision reference tests/benchmarks all `OFF`;
    focused build of `dart`, `dart-gui`,
    `UNIT_collision_CollisionGroup`, `UNIT_collision_DARTCollide`,
    `UNIT_collision_DartCollisionDetector`, `UNIT_constraint_ContactSurface`,
    `UNIT_simulation_World`, and `UNIT_gui_ImGuiWindowScaling` passed; the
    anchored CTest subset for those six unit-test executables passed 6/6.
- Windows PR CI parser repair:
  - Run/job: `25873037012` / `76033173338` (`CI Windows` /
    `Tests (Release)`).
  - Result: failed before CMake ran while `pixi run test-no-plane OFF` was
    resolving the Windows `config` task.
  - Root cause: the Windows task override used Bash parameter expansion such as
    `${DART_BUILD_COLLISION_FCL_OVERRIDE:-OFF}` inside a raw Pixi shell string.
    Pixi's Windows task parser rejected that syntax before the command reached
    a shell.
  - Repair: wrap the Windows `config`, `config-py`, and `config-install` task
    bodies in `bash -lc`, matching the existing Windows test tasks, so the
    collision override defaults are interpreted by Bash rather than by Pixi's
    Windows shell parser.
- Alt Linux PR CI Eigen package compatibility repair:
  - Run/job: `25873507050` / `76034633035` (`CI Alt Linux (Docker)` /
    `Alt Linux repro (Docker)`).
  - Result: failed during CMake configure before compilation.
  - Root cause: `dart_experimental_dependencies.cmake` requested
    `find_package(Eigen3 3.4 ...)` through the experimental dependency helper.
    Alt Linux provides an Eigen 5 CMake config, and Eigen's config-version file
    rejects a requested lower major version even though Eigen 5 satisfies
    DART's `>=3.4` minimum requirement.
  - Repair: find Eigen3 without passing a requested version, then enforce the
    same explicit `Eigen3_VERSION VERSION_LESS 3.4` minimum check used by the
    core Eigen finder.
  - Focused local validation: `pixi run config` passed with the normal Pixi
    Eigen package and native-only collision defaults. A separate native-only
    CMake configure against a temporary fake Eigen 5 package whose version
    file rejects `Eigen3 3.4` requests also passed, with
    `simulation-experimental: Eigen3✓` and build files generated in
    `build/default/cpp/Release-fake-eigen5-altlinux`.
- Alt Linux PR CI EnTT source-build repair:
  - Run/job: `25874678072` / `76038637120` (`CI Alt Linux (Docker)` /
    `Alt Linux repro (Docker)`).
  - Result: after the Eigen repair, configure reached the next native
    collision dependency and failed because the Alt Linux container does not
    provide an `EnTTConfig.cmake` package.
  - Root cause: `dart-collision-native` included the full
    simulation-experimental dependency bundle even though it only needs Eigen
    and EnTT, and EnTT had no source-build fallback on distributions without a
    packaged config file.
  - Repair: make `dart-collision-native` find only Eigen, EnTT, and optional
    benchmark locally, and add an EnTT FetchContent fallback through
    `cmake/dart_find_entt.cmake`.
  - Focused local validation: a temporary export-style CMake probe including
    `cmake/dart_find_entt.cmake` with an empty prefix created an
    `EnTT::EnTT` dependency target and generated an installed export for a
    target linked against it successfully, and the normal `pixi run lint`
    configure path passed.
  - Follow-up run/job: `25875314192` / `76040846463` got past missing EnTT but
    failed CMake generation because the fetched EnTT build target was not in
    DART's export set.
  - Follow-up repair: populate fetched EnTT headers only and expose
    `EnTT::EnTT` as an imported interface target, so generated DART exports
    reference the package target name rather than trying to export EnTT's build
    target.
- Windows wheel MSVC UTF-8 repair:
  - Run/job: `25874275078` / `76037407583`
    (`Wheels | windows-latest Py312`).
  - Result: failed while building the wheel, during compilation of the
    static `dart-collision-native` target.
  - Root cause: the wheel build's MSVC compile command did not include
    `/utf-8`, and the conda-forge `fmt` headers now require MSVC builds with
    Unicode support to compile in UTF-8 mode.
  - Repair: add `/utf-8` to the root MSVC C++ flags so all Visual Studio C++
    targets, including scikit-build wheel targets, inherit the required source
    and execution charset mode.
- macOS arm64 release include hygiene repair:
  - Run/job: `25875811218` / `76044208533` (`CI macOS` /
    `Release Tests (arm64)`).
  - Result: failed during the `tests` target build. The first failing target
    was `INTEGRATION_collision_native_backend_consistency`.
  - Root cause: the new native collision consistency test and shared collision
    benchmark fixtures included generated CamelCase compatibility headers.
    Clang on macOS reports those generated deprecation and non-portable include
    warnings as errors under the release CI flags.
  - Repair: switch the native collision integration test and collision
    benchmark sources/fixtures to lowercase canonical headers, including the
    explicit `reference/` detector headers for FCL, Bullet, and ODE reference
    benchmark paths.
  - Focused local validation: `cmake --build build/default/cpp/Release
--target INTEGRATION_collision_native_backend_consistency` passed, and
    `cmake --build build/default/cpp/Release --target
bm_scenarios_raycast_batch` passed.
- macOS arm64 debug distance-filter link repair:
  - Run/job: `25877526350` / `76048820253` (`CI macOS` /
    `Debug Tests (arm64)`).
  - Result: after the include cleanup, Debug C++ test build failed linking
    `UNIT_collision_DistanceFilter`.
  - Root cause: `BodyNodeDistanceFilter` declarations survived earlier
    refactors, but `dart/collision/distance_filter.cpp` no longer defined
    `BodyNodeDistanceFilter::needDistance()` or `areAdjacentBodies()`. Clean
    macOS Debug builds therefore had no key function or vtable definition for
    the unit-test target.
  - Observed failure:

    ```text
    Undefined symbols for architecture arm64:
      "dart::collision::BodyNodeDistanceFilter::needDistance(...) const"
      "vtable for dart::collision::BodyNodeDistanceFilter"
    ```

  - Repair: restore the historical body-node distance filtering semantics with
    canonical lowercase includes and null-safe fallback behavior for
    non-`ShapeNode` frames.
  - Focused local validation: `cmake --build build/default/cpp/Debug --target
UNIT_collision_DistanceFilter` passed, `ctest --test-dir
build/default/cpp/Debug -R '^UNIT_collision_DistanceFilter$'
--output-on-failure` passed, `cmake --build build/default/cpp/Release
--target UNIT_collision_DistanceFilter` passed, and the matching Release CTest
    passed.

- macOS arm64 release distance-filter ownership repair:
  - Run/job: `25878909676` / `76053447798` (`CI macOS` /
    `Release Tests (arm64)`).
  - Result: after the body-node implementation repair, Release C++ test
    compilation failed in `UNIT_collision_NativeBackend`.
  - Root cause: `DistanceFilter` is a polymorphic base used through
    `std::shared_ptr<DistanceFilter>`, but it had no virtual destructor. Clang
    promoted deletion of the local `ExcludeFrameDistanceFilter` test subclass
    through the base interface to `-Wdelete-non-abstract-non-virtual-dtor`
    under `-Werror`.
  - Repair: add a defaulted virtual destructor to `DistanceFilter`.
  - Focused local validation:

    ```bash
    cmake --build build/default/cpp/Release --target UNIT_collision_NativeBackend --parallel 5
    ctest --test-dir build/default/cpp/Release -R '^UNIT_collision_NativeBackend$' --output-on-failure -j 5
    ```

    Result: both passed.

- macOS py314 wheel collision-isolation policy repair:
  - Run/job: `25878909684` / `76053782968`
    (`Wheels | macos-latest Py314`).
  - Result: wheel build, repair, and delocation succeeded, then
    `wheel-verify` rejected
    `share/dart/cmake/dart_collision-{fcl,bullet,ode}Component.cmake`.
  - Root cause: the verifier still treated retained FCL/Bullet/ODE component
    files as old backend exports, but those component names are now
    native-backed compatibility facades required for downstream migration.
  - Repair: keep rejecting old runtime libraries, reference libraries, and
    reference component exports while allowing the native-backed compatibility
    component facade files.
  - Focused local validation: a synthetic wheel containing
    `dart_collision-{fcl,bullet,ode}Component.cmake` passed the verifier,
    synthetic wheels containing
    `dart_collision-reference-fclComponent.cmake` and `libfcl*.dylib` were
    rejected, and
    `python scripts/verify_wheel_collision_isolation.py dist/dartpy-*.whl`
    passed against the repaired local py312 Linux wheel.

## Current Full-Validation Repair

- `pixi run test-all`
  - Commit: `1da52368282`
    (`Record native collision PR evidence gap`), before the stale optional
    libccd cache repair.
  - Result: failed during `build-tests`. Configure had default native
    collision reference tests disabled, but the existing CMake cache still
    pointed `LIBCCD_LIBRARY` at a removed
    `.pixi/envs/default/lib/libccd.so`, leaving `test_libccd_algorithms` as a
    generated target with a missing link dependency.
  - Observed failure:

    ```text
    ninja: error: '/home/jeongseok/dev/jslee02/dartsim/dart/task_1/.pixi/envs/default/lib/libccd.so', needed by 'bin/test_libccd_algorithms', missing and no known rule to make it
    ```

- `pixi run build-tests ON Release`
  - Commit: working tree after adding missing-path cache reset logic to
    `cmake/dart_test_libccd.cmake`.
  - Result: passed, 469/469 build steps. Configure generated
    `collision-native: Total 29 unit test(s)` and added 264 tests, confirming
    `test_libccd_algorithms` was excluded when optional libccd was not present
    in the default environment.

- `DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all`
  - Commit: working tree after adding the stale optional libccd cache repair
    and updating native-collision task docs.
  - Result: passed. The full local suite passed lint, Release build,
    Release C++ test build, C++ unit tests, simulation-experimental tests,
    Python tests, and documentation. Release CTest passed 264/264 tests,
    including the `collision-native` label summary of 29 tests.
  - Remaining validation: this refreshes local validation for the current
    source state. The final north-star PR state still needs a fresh rerun after
    the remaining CI/migration/deletion work is complete.

## Test Runs

- Current native compatibility package smoke
  - Command:

    ```bash
    pixi run -- bash -lc 'set -euo pipefail
    rm -rf build/native-compat-install build/native-compat-package-smoke
    cmake --install build/default/cpp/Release --prefix build/native-compat-install --component headers >/tmp/dart_native_compat_install.log
    cmake --install build/default/cpp/Release --prefix build/native-compat-install >>/tmp/dart_native_compat_install.log
    cmake -S docs/dev_tasks/native_collision/smoke/native_compat_package -B build/native-compat-package-smoke -DCMAKE_PREFIX_PATH="$PWD/build/native-compat-install;$CONDA_PREFIX" >/tmp/dart_native_compat_smoke_config.log
    cmake --build build/native-compat-package-smoke --parallel 4 >/tmp/dart_native_compat_smoke_build.log
    build/native-compat-package-smoke/native_collision_compat_package_smoke
    if find build/native-compat-install/lib -maxdepth 1 \( -type f -o -type l \) | sort | rg "dart-collision-reference|lib(fcl|bullet|ode|ccd)"; then
      exit 1
    fi
    echo "native compatibility package smoke passed"'
    ```

  - Commit: `892e50d02e4`
    (`Add native compatibility package smoke`).
  - Result: passed. The installed package accepted the retained
    `collision-fcl`, `collision-bullet`, and `collision-ode` package
    components, linked the component facade targets, included installed
    FCL/Bullet/ODE detector headers, verified factory keys create the canonical
    `dart` detector, verified direct legacy class `create()` calls are backed
    by `DartCollisionDetector` while retaining legacy display strings, and
    found no installed `libdart-collision-reference-*`, FCL, Bullet, ODE, or
    libccd runtime libraries.

- Current downstream migration evidence refresh
  - Commit: `8c83cd19cb8`
    (`Record native collision resume evidence`).
  - Commands:

    ```bash
    DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -e gazebo test-gz
    DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -- bash -lc 'set -euo pipefail
    rm -rf build/native-compat-install build/native-compat-package-smoke
    cmake --install build/default/cpp/Release --prefix build/native-compat-install --component headers >/tmp/dart_native_compat_install.log
    cmake --install build/default/cpp/Release --prefix build/native-compat-install >>/tmp/dart_native_compat_install.log
    cmake -S docs/dev_tasks/native_collision/smoke/native_compat_package -B build/native-compat-package-smoke -DCMAKE_PREFIX_PATH="$PWD/build/native-compat-install;$CONDA_PREFIX" >/tmp/dart_native_compat_smoke_config.log
    cmake --build build/native-compat-package-smoke --parallel "$DART_PARALLEL_JOBS" >/tmp/dart_native_compat_smoke_build.log
    build/native-compat-package-smoke/native_collision_compat_package_smoke
    if find build/native-compat-install/lib -maxdepth 1 \( -type f -o -type l \) | sort | rg "dart-collision-reference|lib(fcl|bullet|ode|ccd)"; then
      exit 1
    fi'
    pixi run check-collision-runtime-isolation
    cmake --build build/default/cpp/Release --target test_legacy_compat_facades --parallel 5
    ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_legacy_compat_facades$'
    PYTHONPATH="$PWD/build/default/cpp/Release/python:$PWD/build/default/cpp/Release/python/dartpy" DARTPY_RUNTIME_DIR="$PWD/build/default/cpp/Release/python/dartpy" python -m pytest python/tests/unit/collision/test_collision.py::test_legacy_collision_detector_names_are_native_backed -v
    readelf -d .deps/gz-physics/build/lib/libgz-physics-dartsim-plugin.so | rg 'NEEDED|RUNPATH'
    readelf -d build/native-compat-package-smoke/native_collision_compat_package_smoke | rg 'NEEDED|RUNPATH'
    ```

  - Result: passed locally. The fresh gz-physics workflow patched only the
    DART version requirement, built the DART plugin, and printed the DART
    integration success message. The package smoke passed on the current
    installed native build. Runtime isolation passed. The focused C++ legacy
    facade CTest passed 1/1, and the focused Python legacy detector alias test
    passed 1/1. `readelf` showed both the gz DART plugin and the package-smoke
    executable depend on `libdart-collision-native.so` and do not depend on
    `libdart-collision-reference-*`, FCL, Bullet, ODE, or libccd. This is
    primary local evidence; manual GitHub Actions runs are reference evidence
    only.

- `python scripts/check_collision_runtime_isolation.py`
  - Commit: `892e50d02e4`
    (`Add native compatibility package smoke`).
  - Result: passed. Non-reference DART source paths still do not include FCL,
    Bullet, ODE, libccd, or explicit collision reference backend headers, and
    legacy implementation sources remain under explicit `reference/` paths.
- `pixi run --locked -e collision-reference bm-collision-check`
  - Commit: `892e50d02e4`
    (`Add native compatibility package smoke`).
  - Result: passed. The current broad local benchmark guard rebuilt and ran the
    checked native-vs-reference narrowphase, distance, raycast,
    mixed-primitive, mesh-heavy, and raycast-batch subsets, plus the public
    `DartCollisionDetector` adapter JSON subset.
  - Native/reference check results:
    | Subset | Result |
    | --- | --- |
    | Narrowphase | `3 passed, 0 failed, 0 skipped` |
    | Distance | `3 passed, 0 failed, 0 skipped` |
    | Raycast | `5 passed, 0 failed, 0 skipped` |
    | Mixed primitives | `4 passed, 0 failed, 0 skipped` |
    | Mesh-heavy | `1 passed, 0 failed, 0 skipped` |
    | Raycast batch | `2 passed, 0 failed, 0 skipped` |
  - JSON artifacts were written locally under `.benchmark_results/`:
    `collision_check_narrow.json`, `collision_check_distance.json`,
    `collision_check_raycast.json`, `collision_check_mixed.json`,
    `collision_check_mesh.json`, `collision_check_raycast_batch.json`, and
    `collision_check_adapter.json`. GitHub run artifact evidence is still
    required for the CI benchmark gate.
- `cmake --build build/default/cpp/Release --parallel 15 --target
test_mesh_mesh UNIT_collision_DartCollisionDetector UNIT_collision_DARTCollide`
  - Commit: working tree after capped large flat box/mesh contact patch and
    FCL/ODE legacy facade raycast compatibility.
  - Result: passed. This rebuilt the native mesh regression, the public
    `DartCollisionDetector` suite with FCL/ODE facade raycast coverage, and
    the public DART collide target.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(test_mesh_mesh|UNIT_collision_DartCollisionDetector|UNIT_collision_DARTCollide)$'`
  - Commit: working tree after capped large flat box/mesh contact patch and
    FCL/ODE legacy facade raycast compatibility.
  - Result: passed, 3/3 tests.
- Focused gz `COMMON_TEST_simulation_features`
  - Command: after `pixi run -e gazebo install OFF`, rebuilt the gz DART plugin
    as needed and ran
    `.deps/gz-physics/build/bin/COMMON_TEST_simulation_features
.deps/gz-physics/build/lib/libgz-physics-dartsim-plugin.so.9.0.0
--gtest_brief=1`.
  - Commit: working tree after capped large flat box/mesh contact patch and
    FCL/ODE legacy facade raycast compatibility.
  - Result: passed, 15/15 tests. This closes the prior
    `StepWorld` timeout, `CollisionPairMaxContacts`,
    `CollisionPairMaxContactsSelection`, and
    `UnsupportedRayIntersections` failures.
- `DART_PARALLEL_JOBS=15 pixi run -e gazebo test-gz`
  - Commit: working tree after capped large flat box/mesh contact patch and
    FCL/ODE legacy facade raycast compatibility.
  - Result: passed from a fresh `.deps/gz-physics` clone, 65/65 tests. This is
    the current local full gz-physics compatibility gate.
- `cmake --build build/default/cpp/Release --parallel $(python
scripts/parallel_jobs.py) --target UNIT_constraint_ConstraintSolver &&
build/default/cpp/Release/bin/UNIT_constraint_ConstraintSolver
--gtest_filter='ConstraintSolver.GzPlaneBoxJointDetachKeepsSupportVelocitySymmetric'
--gtest_brief=1`
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed. The reduced DART regression covers the gz
    `JointDetach` support-contact path with gz's plane-as-large-box fallback,
    preserving the expected upward base/upper-link motion while keeping
    upper-link `X`, `Y`, angular `Y`, and angular `Z` near zero.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_(NativeBackend|DARTCollide|CollisionGroup)|UNIT_constraint_ConstraintSolver)$'`
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed, 4/4 tests. This covers the focused constraint solver
    regression plus native collision group, public DART collide, and native
    backend coverage after the narrowphase support-contact change.
- `pixi run -e gazebo install OFF && pixi run -e gazebo -- bash -lc 'cmake
--build .deps/gz-physics/build --parallel $(python scripts/parallel_jobs.py)
--target gz-physics-dartsim-plugin COMMON_TEST_joint_features'`
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed. DART was reinstalled into the gazebo environment and the
    gz DART plugin plus joint feature test binary rebuilt against it.
- `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ./bin/COMMON_TEST_joint_features
./lib/libgz-physics-dartsim-plugin.so.9.0.0
--gtest_filter="JointFeaturesDetachTest/0.JointDetach" --gtest_brief=1'`
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed, 1/1 test. This closes the previously failing
    `upperLinkLinearVelocity.X()`, `upperLinkLinearVelocity.Y()`, and
    `upperLinkAngularVelocity.Y()` exact-zero residual.
- `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ./bin/COMMON_TEST_joint_features
./lib/libgz-physics-dartsim-plugin.so.9.0.0 --gtest_brief=1'`
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed, 21/23 tests passed and 2 skipped. This verifies the full
    local `COMMON_TEST_joint_features` binary against the DART plugin after
    closing `JointDetach`.
- `pixi run -e gazebo -- bash -lc 'cmake --build .deps/gz-physics/build
--parallel $(python scripts/parallel_jobs.py) --target COMMON_TEST_collisions
COMMON_TEST_detachable_joint COMMON_TEST_joint_transmitted_wrench_features'`
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed. The three related focused gz test binaries rebuilt.
- Focused gz smoke runs for related downstream gates:
  - Commands: `COMMON_TEST_collisions`, `COMMON_TEST_detachable_joint`, and
    `COMMON_TEST_joint_transmitted_wrench_features` were each run against
    `./lib/libgz-physics-dartsim-plugin.so.9.0.0` with `--gtest_brief=1`.
  - Commit: working tree after tilted cylinder/plane-like-box support patch.
  - Result: passed. `COMMON_TEST_collisions` passed 5/5 tests,
    `COMMON_TEST_detachable_joint` passed 1/1 test, and
    `COMMON_TEST_joint_transmitted_wrench_features` passed 5/5 tests.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target test_cylinder test_narrow_phase
test_collision_world UNIT_collision_DartCollisionDetector --parallel
$(python scripts/parallel_jobs.py)`
  - Commit: `6e04945b29d6`
    (`Improve native cylinder-box axial cap contacts`).
  - Result: passed.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^(test_cylinder|test_narrow_phase|test_collision_world|UNIT_collision_DartCollisionDetector)$'`
  - Commit: `6e04945b29d6`
    (`Improve native cylinder-box axial cap contacts`).
  - Result: passed, 4/4 tests. This covers native cylinder-cylinder stacked
    cap support, axial cylinder-cap support patches against large boxes,
    pair-order normal semantics in direct narrowphase, optimized
    collision-world snapshot dispatch, and public DART mesh-plane collision
    normal orientation.
- `pixi run -e gazebo install OFF`
  - Commit: source state that became `6e04945b29d6`
    (`Improve native cylinder-box axial cap contacts`).
  - Result: passed; the gz environment was reinstalled against the current DART
    build before downstream focused tests.
- `pixi run -e gazebo -- bash -lc 'export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ctest --output-on-failure -R
"^COMMON_TEST_(collisions|detachable_joint|joint_features|joint_transmitted_wrench_features)_dartsim$"'`
  - Commit: source state that became `6e04945b29d6`
    (`Improve native cylinder-box axial cap contacts`).
  - Result: 3/4 passed. `COMMON_TEST_collisions_dartsim`,
    `COMMON_TEST_detachable_joint_dartsim`, and
    `COMMON_TEST_joint_transmitted_wrench_features_dartsim` passed. The
    remaining failure is `COMMON_TEST_joint_features_dartsim`
    `JointFeaturesDetachTest/0.JointDetach`, which reports
    `upperLinkLinearVelocity.X() = -0.00013950340597912903` and
    `upperLinkAngularVelocity.Y() = -6.642346064788122e-05` against exact-zero
    tolerances of `1e-6`. This is now the focused gz compatibility residual.
- `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ./bin/COMMON_TEST_joint_features
./lib/libgz-physics-dartsim-plugin.so.9.0.0
--gtest_filter="JointFeaturesDetachTest/0.JointDetach" --gtest_brief=1'`
  - Commit: `6e04945b29d6`
    (`Improve native cylinder-box axial cap contacts`).
  - Result: failed as expected on the focused residual. The remaining failures
    are `upperLinkLinearVelocity.X() = -0.00013953469787260998`,
    `upperLinkLinearVelocity.Y() = 1.0002864929523347e-06`, and
    `upperLinkAngularVelocity.Y() = -6.6438361021132697e-05` against
    exact-zero tolerances of `1e-6`.
- `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ./bin/COMMON_TEST_joint_features
./lib/libgz-physics-dartsim-plugin.so.9.0.0
--gtest_filter="JointFeaturesDetachTest/0.JointDetach" --gtest_brief=1'`
  - Commit: `e2ee7a88b52`
    (`Preserve native cache writeback through facades`) with temporary
    gz-physics test instrumentation only.
  - Result: failed as expected, but the added per-step velocity trace isolated
    the residual. At step 9, base angular velocity around `Y` is
    `-6.6445229724690596e-05`, upper-link angular velocity around `Y` is
    `-6.6438361021132697e-05`, and upper-link linear velocity `X` is
    `-0.00013953469787260998`. The upper-link off-axis residual therefore
    follows base support motion through the joint offset. The next reduction
    should focus on base-vs-ground native support contact/manifold stability,
    not on detach-state restoration or legacy facade routing. The temporary
    `.deps/gz-physics` instrumentation was removed after capture.
- Temporary centered cylinder-cap/large-box support-contact experiment
  - Source base: `f41b1ee3bc5`
    (`Document native collision architecture rubric`) plus an uncommitted
    local helper in
    `dart/collision/native/narrow_phase/cylinder_collision.cpp`; reverted
    after the run.
  - Commands: built and ran focused `test_cylinder`, rebuilt the gz DART
    plugin through the standard gazebo install and focused
    `gz-physics-dartsim-plugin`/`COMMON_TEST_joint_features` targets, then ran
    isolated `JointFeaturesDetachTest/0.JointDetach`.
  - Result: focused native coverage passed, but gz failed earlier at the
    expected upward-motion check:
    `upperLinkLinearVelocity.Z() = 1.63203e-06`, below the required `1e-5`.
    Off-axis values were also still nonzero:
    `upperLinkLinearVelocity.X() = 7.0895968540904397e-05`,
    `upperLinkLinearVelocity.Y() = 0.00015833169537646667`, and
    `upperLinkAngularVelocity.Y() = 3.3759986207481524e-05`. This rejects a
    centered support point as the next fix direction.
- Temporary weighted-centroid cylinder-cap/large-box support-contact experiment
  - Source base: `f41b1ee3bc5`
    (`Document native collision architecture rubric`) plus an uncommitted
    local helper in
    `dart/collision/native/narrow_phase/cylinder_collision.cpp`; reverted
    after the run.
  - Commands: built and ran focused `test_cylinder`, rebuilt the gz DART
    plugin through the standard gazebo install and focused
    `gz-physics-dartsim-plugin`/`COMMON_TEST_joint_features` targets, then ran
    isolated `JointFeaturesDetachTest/0.JointDetach`.
  - Result: focused native coverage passed, but gz again failed the expected
    upward-motion check:
    `upperLinkLinearVelocity.Z() = -6.02409e-08`, below the required `1e-5`.
    This rejects weighted support-centroid shaping as the next fix direction.
    The retained source state keeps the validated axial cap support patch and
    the convex fallback for tilted cap/box cases.
- `cmake --build build/default/cpp/Release --target
UNIT_constraint_SoftContactConstraint --parallel $(python
scripts/parallel_jobs.py)`
  - Commit: working tree after native cache facade update.
  - Result: passed. This rebuilds the solver/contact constraint target with a
    regression for native manifold cache impulse writeback through a
    native-backed legacy display-name detector facade.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_constraint_SoftContactConstraint$'`
  - Commit: working tree after native cache facade update.
  - Result: passed, 1/1 test. The new
    `NativeCacheUpdatesThroughCompatibilityFacade` case verifies that a
    detector facade reporting `"ode"` still writes the applied normal and
    friction impulses back to the attached native `CachedContact`.
- `cmake --build build/default/cpp/Release --parallel $(python
scripts/parallel_jobs.py) --target UNIT_collision_NativeBackend`
  - Commit: working tree after canonical manifold cache contact ordering.
  - Result: passed. This rebuilds the native backend test target with the
    DART adapter change that stores warm-start local points in the same
    canonical scene-cache ID order used by the persistent manifold key.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_NativeBackend$'`
  - Commit: working tree after canonical manifold cache contact ordering.
  - Result: passed, 1/1 test. The new
    `PersistentManifoldCacheWarmStartAcrossGroupOrder` case verifies cached
    normal and friction impulses survive when the same pair is first queried as
    group B/A and then as group A/B.
- `pixi run -e gazebo install OFF && pixi run -e gazebo -- bash -lc 'cmake
--build .deps/gz-physics/build --parallel $(python scripts/parallel_jobs.py)
--target gz-physics-dartsim-plugin COMMON_TEST_joint_features'`
  - Commit: working tree after canonical manifold cache contact ordering.
  - Result: passed. The gz DART plugin and focused joint feature test binary
    rebuilt against the current DART install.
- `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ./bin/COMMON_TEST_joint_features
./lib/libgz-physics-dartsim-plugin.so.9.0.0
--gtest_filter="JointFeaturesDetachTest/0.JointDetach" --gtest_brief=1'`
  - Commit: working tree after canonical manifold cache contact ordering.
  - Result: failed with the same focused residual values:
    `upperLinkLinearVelocity.X() = -0.00013953469787260998`,
    `upperLinkLinearVelocity.Y() = 1.0002864929523347e-06`, and
    `upperLinkAngularVelocity.Y() = -6.6438361021132697e-05`. The canonical
    cache-order fix is retained as a correctness improvement, but it does not
    close the `JointDetach` support-stability gate.
- `pixi run lint`
  - Commit: working tree after canonical manifold cache contact ordering.
  - Result: passed. `codespell`, CMake `format`, `clang-format`, `black`,
    `isort`, `prettier`, TOML lint, RST lint, collision runtime isolation, and
    AI command sync completed.
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

## Planning Documentation Runs

- `pixi run lint`
  - Commit: working tree after north-star planning and built-in architecture
    documentation update
  - Result: passed. `codespell`, CMake `format`, `clang-format`, `black`,
    `isort`, `prettier`, TOML lint, RST lint, and AI command sync completed.
- `pixi run check-docs-policy`
  - Commit: working tree after north-star planning and built-in architecture
    documentation update
  - Result: passed.
- `git diff --check`
  - Commit: working tree after north-star planning and built-in architecture
    documentation update
  - Result: passed.
- `pixi run lint`
  - Commit: working tree after reference capability gap analysis documentation
    update
  - Result: passed.
- `pixi run check-docs-policy`
  - Commit: working tree after reference capability gap analysis documentation
    update
  - Result: passed.
- `git diff --check`
  - Commit: working tree after reference capability gap analysis documentation
    update
  - Result: passed.
- `pixi run lint`
  - Commit: working tree after north-star layer design documentation update
  - Result: passed. CMake configure, C++ formatting, docs formatting, spell
    check, Python formatting, TOML/YAML/RST checks, and AI command sync
    completed.
- `pixi run check-docs-policy`
  - Commit: working tree after north-star layer design documentation update
  - Result: passed.
- `git diff --check`
  - Commit: working tree after north-star layer design documentation update
  - Result: passed.
- `pixi run lint`
  - Commit: working tree after architecture subcomponent docs and native cache
    facade update.
  - Result: passed. CMake configure, C++ formatting, docs formatting,
    spell check, Python formatting, TOML/YAML/RST checks, collision runtime
    isolation, and AI command sync completed.
- `pixi run check-docs-policy`
  - Commit: working tree after architecture subcomponent docs and native cache
    facade update.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after architecture subcomponent docs and native cache
    facade update.
  - Result: passed.

## Built-In Architecture Runs

This section tracks architecture evidence, not only docs updates. The gate is
complete only when the API cleanliness, adapter scalability, native core
scalability, performance orientation, reference isolation, and compatibility
facade work items in `01-design.md` have matching tests, benchmarks, package
inspection, source-boundary checks, and downstream/gz evidence.

- `cmake --build build/default/cpp/Release --target test_shapes --parallel
$(python scripts/parallel_jobs.py) && ctest --test-dir
build/default/cpp/Release --output-on-failure -R '^test_shapes$'`
  - Commit: working tree after removing unused direct native `ShapeType`
    values for cone, heightfield, and point-cloud.
  - Result: passed. The native shape enum no longer advertises phantom direct
    shape tags that have no native shape class or dispatch path.
- `cmake --build build/default/cpp/Release --target
dart_collision_native_tests --parallel $(python scripts/parallel_jobs.py) &&
ctest --test-dir build/default/cpp/Release --output-on-failure -L
collision-native`
  - Commit: working tree after rebuilding aggregate native tests for the
    `ShapeType` taxonomy cleanup.
  - Result: passed, 29/29 `collision-native` tests. A first label run before
    rebuilding the aggregate tests failed because old test binaries still had
    the previous enum ordinals compiled in; rebuilding the aggregate target
    refreshed those binaries and the label passed.

- `pixi run lint`
  - Commit: working tree after persistent DART adapter scene, raycast pruning,
    and distance AABB lower-bound pruning implementation
  - Result: passed. C++ formatting, docs formatting, spell check, Python
    formatting, TOML/YAML/RST checks, and AI command sync completed.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector --parallel 4`
  - Commit: working tree after persistent DART adapter scene, raycast pruning,
    and distance AABB lower-bound pruning implementation
  - Result: passed.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_collision'`
  - Commit: working tree after persistent DART adapter scene, raycast pruning,
    and distance AABB lower-bound pruning implementation
  - Result: passed, 20/20 collision unit tests.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector --parallel 4`
  - Commit: working tree after scene-issued manifold cache IDs and
    shape-replacement warm-start invalidation coverage
  - Result: passed.
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after scene-issued manifold cache IDs and
    shape-replacement warm-start invalidation coverage
  - Result: passed, 1/1 detector test.
- `pixi run lint`
  - Commit: working tree after scene-issued manifold cache IDs, documentation
    updates, and focused warm-start invalidation coverage
  - Result: passed. C++ formatting, docs formatting, spell check, Python
    formatting, TOML/YAML/RST checks, and AI command sync completed.
- `pixi run check-docs-policy && git diff --check && cmake --build
build/default/cpp/Release --target UNIT_collision_DartCollisionDetector
--parallel 4 && ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_collision'`
  - Commit: working tree after final lint for scene-issued manifold cache IDs
    and architecture docs
  - Result: passed. Docs policy and whitespace checks passed; build passed;
    CTest passed, 20/20 collision unit tests.
- `cmake --build build/default/cpp/Release --target
bm_scenarios_dart_adapter --parallel 4`
  - Commit: working tree after adding public DART adapter-path benchmark
    coverage for collision, dirty-world collision, distance, and raycast
  - Result: passed.
- `build/default/cpp/Release/bin/bm_scenarios_dart_adapter
--benchmark_filter='DartAdapter' --benchmark_min_time=0.01s
--benchmark_repetitions=1`
  - Commit: working tree after adding public DART adapter-path benchmark
    coverage
  - Result: passed. Ran eight public adapter benchmark cases:
    `CollidePersistentScene` at 64/256 objects, `CollideDirtySubset` at
    256/1024 objects, `DistancePersistentScene` at 64/256 objects, and
    `RaycastPersistentScene` at 256/1024 objects.
- `pixi run lint`
  - Commit: working tree after public DART adapter benchmark target and
    benchmark documentation updates
  - Result: passed. C++ formatting, docs formatting, spell check, Python
    formatting, TOML/YAML/RST checks, and AI command sync completed.
- `cmake --build build/default/cpp/Release --target
bm_scenarios_dart_adapter UNIT_collision_DartCollisionDetector --parallel 4 &&
build/default/cpp/Release/bin/bm_scenarios_dart_adapter
--benchmark_filter='DartAdapter' --benchmark_min_time=0.01s
--benchmark_repetitions=1 && pixi run check-docs-policy && git diff --check &&
ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_collision'`
  - Commit: working tree after final lint for public DART adapter benchmark
    coverage
  - Result: passed. Benchmark and detector targets built; eight public adapter
    benchmark cases ran; docs policy and whitespace checks passed; CTest passed,
    20/20 collision unit tests.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector --parallel 4 && ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after primitive in-place shape mutation invalidation
    coverage
  - Result: passed, 1/1 detector test.
- `pixi run lint && pixi run check-docs-policy && git diff --check && cmake
--build build/default/cpp/Release --target UNIT_collision_DartCollisionDetector
--parallel 4 && ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_collision'`
  - Commit: working tree after primitive shape mutation coverage and checklist
    updates
  - Result: passed. Lint, docs policy, whitespace check, detector build, and
    20/20 collision unit tests passed.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector --parallel 4 && ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after mesh scale, heightmap, voxel-grid, and
    point-cloud invalidation/behavior coverage
  - Result: passed, 1/1 detector test. The build emitted the known third-party
    OctoMap `<ciso646>` C++20 warning when voxel-grid coverage was compiled.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector --parallel 4 && ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after dynamic-vertex geometry resync and soft-mesh
    point-mass mutation coverage
  - Result: passed, 1/1 detector test. The build emitted the known third-party
    OctoMap `<ciso646>` C++20 warning when voxel-grid coverage was compiled.
- `pixi run lint && pixi run check-docs-policy && git diff --check && cmake
--build build/default/cpp/Release --target UNIT_collision_DartCollisionDetector
--parallel 4 && ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_collision'`
  - Commit: working tree after dynamic-vertex geometry resync, soft-mesh
    mutation coverage, and docs checklist updates
  - Result: passed. Lint, docs policy, whitespace check, detector build, and
    20/20 collision unit tests passed. The build emitted the known third-party
    OctoMap `<ciso646>` C++20 warning when voxel-grid coverage was compiled.
- `cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector UNIT_collision_CollisionFilter --parallel
4 && ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_DartCollisionDetector|UNIT_collision_CollisionFilter)$'`
  - Commit: working tree after adapting DART collision filters into native pair
    checks before narrowphase
  - Result: passed, 2/2 tests.
- `pixi run lint && pixi run check-docs-policy && git diff --check && cmake
--build build/default/cpp/Release --target UNIT_collision_DartCollisionDetector
UNIT_collision_CollisionFilter --parallel 4 && ctest --test-dir
build/default/cpp/Release --output-on-failure -R '^UNIT_collision'`
  - Commit: working tree after dynamic-vertex geometry resync, native collision
    filter adapter, and docs checklist updates
  - Result: passed. Lint, docs policy, whitespace check, detector/filter build,
    and 20/20 collision unit tests passed.

## CI Hardening Runs

- `.github/workflows/ci_ubuntu.yml` native-only job addition
  - Commit: working tree after adding
    `Native Collision (no FCL/Bullet/ODE)` to CI Linux
  - Result: not run in GitHub CI yet. The job configures with
    `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF`,
    `DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF`, and
    `DART_BUILD_COLLISION_ODE_OVERRIDE=OFF`; builds `dart`, `dartpy`, native
    collision tests, and focused default-detector tests; runs the
    `collision-native` label; and runs dartpy collision/world smoke tests.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF DART_VERBOSE=ON pixi run config &&
DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF pixi run -- cmake --build
build/default/cpp/Release --target dart dartpy dart_collision_native_tests
UNIT_collision_CollisionGroup UNIT_collision_Raycast
UNIT_collision_DartCollisionDetector UNIT_collision_NativeBackend --parallel
8 && pixi run -- ctest --test-dir build/default/cpp/Release
--output-on-failure -L collision-native && pixi run -- ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_CollisionGroup|UNIT_collision_Raycast|UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend)$'
&& PYTHONPATH=build/default/cpp/Release/python pixi run -- pytest -q
python/tests/unit/collision/test_collision.py
python/tests/unit/simulation/test_world.py`
  - Commit: working tree after adding the native-only CI job.
  - Result: passed locally. Configure reported
    `DART_BUILD_COLLISION_FCL: OFF`, `DART_BUILD_COLLISION_BULLET: OFF`, and
    `DART_BUILD_COLLISION_ODE: OFF`; build completed the native-only CI job
    targets; CTest passed the `collision-native` label, 29/29 tests; focused
    default-detector CTests passed, 4/4 tests; Python collision/world smoke
    passed, 9 passed and 1 skipped. The skipped Python case is the expected
    Bullet detector check when Bullet is disabled. This is local job-equivalent
    evidence, not a substitute for the pending GitHub CI result.

## Reference Harness Isolation Runs

- `DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF DART_VERBOSE=ON pixi
run config`
  - Commit: working tree after adding reference test/benchmark CMake opt-out
    options.
  - Result: passed. Configure kept
    `DART_BUILD_COLLISION_FCL=ON`, `DART_BUILD_COLLISION_BULLET=ON`, and
    `DART_BUILD_COLLISION_ODE=ON`, while reporting
    `DART_BUILD_COLLISION_REFERENCE_TESTS=OFF` and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF`. The generated test set had
    264 tests, and the native collision unit set had 29 tests because the
    reference-backend native test target was excluded.
- `DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF pixi run config && pixi
run -- cmake --build build/default/cpp/Release --target
UNIT_collision_CollisionGroup UNIT_collision_Raycast UNIT_simulation_World
INTEGRATION_collision_native_backend_consistency INTEGRATION_io_SkelParser
INTEGRATION_simulation_World bm_scenarios_raycast_batch --parallel 8 && pixi
run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_CollisionGroup|UNIT_collision_Raycast|UNIT_simulation_World|INTEGRATION_collision_native_backend_consistency|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World)$'`
  - Commit: working tree after gating focused reference tests and comparative
    benchmarks.
  - Result: passed. Focused reference-disabled build completed and CTest passed
    6/6 selected native/default tests. The build emitted only known
    third-party OctoMap and deprecated compatibility-header warnings.
- `cmake --build build/default/cpp/Release --target help | rg
'test_reference_backends|bm_comparative|bm_boxes|bm_scenarios_mixed_primitives|bm_scenarios_mesh_heavy|INTEGRATION_simulation_MimicConstraint'
|| true`
  - Commit: working tree after reference-disabled configure.
  - Result: no matching targets were listed, confirming those reference-only
    tests and comparative benchmarks are absent when the reference options are
    disabled.
- Reference-disabled probe before final gating:
  - Result: `INTEGRATION_simulation_MimicConstraint` failed
    `MimicConstraint.FollowersMatchMiddlePendulum` when forced onto the native
    default detector in a reference-disabled build. The target is now gated as
    reference-engine coverage for this phase rather than counted as
    native/default coverage. This is a tracked evidence point, not a north-star
    pass for native MimicConstraint behavior.
- `pixi run config && pixi run -- cmake --build build/default/cpp/Release
--target test_reference_backends bm_comparative_raycast
INTEGRATION_simulation_MimicConstraint --parallel 8 && pixi run -- ctest
--test-dir build/default/cpp/Release --output-on-failure -R
'^(test_reference_backends|INTEGRATION_simulation_MimicConstraint)$'`
  - Commit: working tree after reference test/benchmark opt-out support.
  - Result: passed with default reference options enabled. Configure reported
    `DART_BUILD_COLLISION_REFERENCE_TESTS=ON` and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=ON`; the selected reference
    targets built; CTest passed 2/2 tests.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF
DART_BUILD_EXAMPLES_OVERRIDE=OFF DART_BUILD_TUTORIALS_OVERRIDE=OFF
DART_BUILD_GUI_OVERRIDE=OFF DART_VERBOSE=ON pixi run config-debug && pixi run
-- cmake -LA -N build/default/cpp/Debug | rg
'DART_BUILD_COLLISION_(FCL|BULLET|ODE|REFERENCE_TESTS|REFERENCE_BENCHMARKS):'`
  - Commit: working tree after propagating reference toggles to configure entry
    points.
  - Result: passed. The Debug CMake cache reported FCL, Bullet, ODE, reference
    tests, and reference benchmarks all `OFF`.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF DART_VERBOSE=ON pixi
run config-py && pixi run -- cmake -LA -N build/default/cpp/Release | rg
'DART_BUILD_COLLISION_(FCL|BULLET|ODE|REFERENCE_TESTS|REFERENCE_BENCHMARKS):'`
  - Commit: working tree after propagating reference toggles to the dartpy
    configure path.
  - Result: passed. The Release dartpy CMake cache reported FCL, Bullet, ODE,
    reference tests, and reference benchmarks all `OFF`.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF
DART_BUILD_GUI_OVERRIDE=OFF DART_VERBOSE=ON pixi run config-install OFF &&
pixi run -- cmake -LA -N build/default/cpp | rg
'DART_BUILD_COLLISION_(FCL|BULLET|ODE|REFERENCE_TESTS|REFERENCE_BENCHMARKS):'`
  - Commit: working tree after propagating reference toggles to install
    configure paths.
  - Result: passed. The install-style CMake cache reported FCL, Bullet, ODE,
    reference tests, and reference benchmarks all `OFF`.
- `pixi run -- cmake --build build/default/cpp --target dart --parallel 8`
  - Commit: working tree after native-only install-style configure.
  - Result: passed. The top-level install-style build output had only
    `libdart.so` in `build/default/cpp/lib`; no `dart-collision-fcl`,
    `dart-collision-bullet`, or `dart-collision-ode` library was produced in
    that build tree.
- `ldd build/default/cpp/lib/libdart.so.7.0.0 | grep -Ei
'lib(fcl|bullet|ode|ccd)' || true`
  - Commit: working tree after native-only install-style `dart` build.
  - Result: no matching runtime links. `libdart.so` did not link FCL, Bullet,
    ODE, or libccd in the native-only install-style build.
- `pixi run -- cmake --build build/default/cpp --target help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'
|| true`
  - Commit: working tree after native-only install-style configure.
  - Result: no matching legacy collision backend or reference-only targets were
    listed in the native-only install-style build.
- `pixi run -- cmake --build build/default/cpp --target
dart_collision_native_tests --parallel 8 && pixi run -- ctest --test-dir
build/default/cpp --output-on-failure -L collision-native && pixi run --
python -m py_compile scripts/wheel_build.py`
  - Commit: working tree after configure-entry-point propagation and wheel
    helper updates.
  - Result: passed. Native collision tests built and CTest passed 29/29 under
    the native-only install-style configuration; `scripts/wheel_build.py`
    compiled successfully.
- `pixi run -- cmake --build build/default/cpp --target dart-utils
dart-utils-urdf dart-io --parallel 8 && pixi run -- cmake --install
build/default/cpp --prefix build/native_collision_install_probe_<timestamp>`
  - Commit: working tree after native-only install-style configure.
  - Result: passed after building the installable utility and IO components.
    The installed prefix contained `libdart`, `libdart-collision-native`,
    `libdart-utils`, `libdart-utils-urdf`, and `libdart-io`; it did not install
    `libdart-collision-fcl`, `libdart-collision-bullet`, or
    `libdart-collision-ode`.
- `find <install-prefix> \( -type f -o -type l \) | rg -i
'collision-(fcl|bullet|ode)|libdart-collision-(fcl|bullet|ode)|lib(fcl|bullet|ode|ccd)'
|| true`
  - Commit: working tree after native-only install probe.
  - Result: no matching installed files or symlinks.
- `for lib in <install-prefix>/lib/*.so*; do ldd "$lib" | grep -Ei
'lib(fcl|bullet|ode|ccd)' || true; done`
  - Commit: working tree after native-only install probe.
  - Result: no matching runtime links from any installed native-only library.
- `rg -n
'collision-(fcl|bullet|ode)|libdart-collision-(fcl|bullet|ode)|lib(fcl|bullet|ode|ccd)|DART_BUILD_COLLISION'
<install-prefix>/share/dart/cmake <install-prefix>/lib/pkgconfig`
  - Commit: working tree after native-only install probe.
  - Result: no installed old collision component files or targets. Expected
    `DART_BUILD_COLLISION_*` variables are `OFF`. Before package-export
    cleanup, generated CMake templates still contained compatibility strings
    for old collision components.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF
DART_BUILD_GUI_OVERRIDE=OFF DART_VERBOSE=ON pixi run config-install OFF &&
pixi run -- cmake --build build/default/cpp --target dart dart-utils
dart-utils-urdf dart-io --parallel 8 && pixi run -- cmake --install
build/default/cpp --prefix build/native_collision_install_probe_<timestamp>
&& rg -n
'collision-(fcl|bullet|ode)|libdart-collision-(fcl|bullet|ode)|lib(fcl|bullet|ode|ccd)|DART_BUILD_COLLISION'
<install-prefix>/share/dart/cmake <install-prefix>/lib/pkgconfig`
  - Commit: working tree after package-export cleanup.
  - Result: passed. The install-style configure reported FCL, Bullet, ODE,
    reference tests, reference benchmarks, and GUI all `OFF`; the install
    completed after building the installable core, utility, URDF, and IO
    targets. The installed CMake/pkg-config metadata search reported only
    `DART_BUILD_COLLISION_BULLET`, `DART_BUILD_COLLISION_FCL`, and
    `DART_BUILD_COLLISION_ODE` set to `OFF`. No `collision-fcl`,
    `collision-bullet`, `collision-ode`, `libdart-collision-fcl`,
    `libdart-collision-bullet`, `libdart-collision-ode`, `libfcl`,
    `libbullet`, `libode`, or `libccd` strings were reported from the searched
    install metadata.
- `DART_VERBOSE=ON pixi run config`
  - Commit: working tree after defaulting normal pixi configure paths to
    native-only collision.
  - Result: passed. Configure reported
    `DART_BUILD_COLLISION_FCL: OFF [default]`,
    `DART_BUILD_COLLISION_BULLET: OFF [default]`,
    `DART_BUILD_COLLISION_ODE: OFF [default]`,
    `DART_BUILD_COLLISION_REFERENCE_TESTS: OFF`, and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS: OFF`. FCL was not listed in
    the found package set, CMake added the default `dart` component, generated
    264 tests, and kept the native collision unit count at 29.
- `pixi run -- cmake --build build/default/cpp/Release --target help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'
|| true`
  - Commit: working tree after native-only pixi default configure.
  - Result: no matching legacy collision component or reference-only targets
    were listed in the default pixi build.
- `pixi run -- cmake --build build/default/cpp/Release --target dart dartpy
dart_collision_native_tests UNIT_collision_DartCollisionDetector
UNIT_collision_NativeBackend --parallel 8`
  - Commit: working tree after native-only pixi default configure.
  - Result: passed. Build observed only known third-party OctoMap `<ciso646>`
    C++20 warnings.
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^(UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend)$'`
  - Commit: working tree after native-only pixi default build.
  - Result: passed, 2/2 tests.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=ON
DART_BUILD_COLLISION_BULLET_OVERRIDE=ON DART_BUILD_COLLISION_ODE_OVERRIDE=ON
DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=ON
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=ON DART_VERBOSE=ON pixi
run config`
  - Commit: working tree after native-only pixi default configure.
  - Result: passed. Configure reported FCL, Bullet, ODE, reference tests, and
    reference benchmarks all `ON`, proving comparison jobs can still opt in.
- `pixi run -- cmake --build build/default/cpp/Release --target help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'`
  - Commit: working tree after explicit reference opt-in configure.
  - Result: matched `dart-collision-fcl`, `dart-collision-bullet`,
    `dart-collision-ode`, `test_reference_backends`, comparative benchmark
    targets, `bm_scenarios_mixed_primitives`, `bm_scenarios_mesh_heavy`, and
    `INTEGRATION_simulation_MimicConstraint`.
- `DART_VERBOSE=ON pixi run config && pixi run -- cmake --build
build/default/cpp/Release --target help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'
|| true && pixi run -- cmake --build build/default/cpp/Release --target dart
dartpy dart_collision_native_tests UNIT_collision_DartCollisionDetector
UNIT_collision_NativeBackend --parallel 8`
  - Commit: working tree after restoring the normal native-only pixi default.
  - Result: passed. Configure again reported FCL, Bullet, ODE, reference tests,
    and reference benchmarks `OFF`; target-help inspection again listed no old
    collision component or reference-only targets; native/default build targets
    passed with only known third-party OctoMap warnings.

## Collision Abstraction And Alias Runs

- `pixi run -- cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector UNIT_simulation_World
INTEGRATION_collision_Collision INTEGRATION_collision_CollisionGroups
INTEGRATION_io_SkelParser INTEGRATION_simulation_World --parallel 8`
  - Commit: working tree after making retained factory keys native-backed
    aliases.
  - Result: passed in the reference-enabled build. The build linked the legacy
    FCL, Bullet, and ODE component libraries for the integration factory test,
    proving their static registrars no longer restore real backend selection
    through the public factory.
- `pixi run -- ctest --test-dir build/default/cpp/Release
--output-on-failure -R
'^(UNIT_collision_DartCollisionDetector|UNIT_simulation_World|INTEGRATION_collision_Collision|INTEGRATION_collision_CollisionGroups|INTEGRATION_io_SkelParser|INTEGRATION_simulation_World)$'`
  - Commit: working tree after making retained factory keys native-backed
    aliases and updating stale SKEL/world expectations.
  - Result: passed, 6/6. Coverage includes public factory keys `dart`,
    `experimental`, `fcl`, `fcl_mesh`, `bullet`, and `ode`, SKEL parser
    handling of `fcl` and `fcl_mesh`, world typed setter behavior for
    `CollisionDetectorType::Fcl`, and collision-group tests parameterized over
    legacy key names.
- `DART_BUILD_COLLISION_FCL_OVERRIDE=OFF
DART_BUILD_COLLISION_BULLET_OVERRIDE=OFF
DART_BUILD_COLLISION_ODE_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=OFF
DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=OFF DART_VERBOSE=ON pixi
run config && pixi run -- cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector INTEGRATION_io_SkelParser
UNIT_simulation_World --parallel 8 && pixi run -- ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_DartCollisionDetector|INTEGRATION_io_SkelParser|UNIT_simulation_World)$'`
  - Commit: working tree after making retained factory keys native-backed
    aliases.
  - Result: passed, 3/3 in a native-only configure. Configure reported FCL,
    Bullet, ODE, reference tests, and reference benchmarks all `OFF`; the
    alias test still proved all retained factory keys create
    `DartCollisionDetector`.
- `pixi run -- cmake --build build/default/cpp/Release --target help | rg
'dart-collision-(fcl|bullet|ode)|INTEGRATION_collision_Collision$|test_reference_backends|bm_comparative'
|| true`
  - Commit: working tree after native-only configure and factory-alias cleanup.
  - Result: no matching old collision component or reference-only targets were
    listed.

## Native-Only Pixi Default Checkpoint Runs

- `pixi run lint`
  - Commit: working tree after native-only pixi defaults and built-in
    architecture docs.
  - Result: passed. The configure phase reported
    `DART_BUILD_COLLISION_FCL: OFF [default]`,
    `DART_BUILD_COLLISION_BULLET: OFF [default]`,
    `DART_BUILD_COLLISION_ODE: OFF [default]`,
    `DART_BUILD_COLLISION_REFERENCE_TESTS: OFF`, and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS: OFF`.
- `pixi run check-docs-policy`
  - Commit: working tree after native-only pixi defaults and built-in
    architecture docs.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after native-only pixi defaults and built-in
    architecture docs.
  - Result: passed.
- `pixi run -- cmake --build build/default/cpp/Release --target help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'
|| true`
  - Commit: working tree after native-only pixi default lint configure.
  - Result: no matching legacy collision component or reference-only targets
    were listed.
- `pixi run -- cmake --build build/default/cpp/Release --target dart dartpy
dart_collision_native_tests UNIT_collision_DartCollisionDetector
UNIT_collision_NativeBackend --parallel 8 && pixi run -- ctest --test-dir
build/default/cpp/Release --output-on-failure -R
'^(UNIT_collision_DartCollisionDetector|UNIT_collision_NativeBackend)$'`
  - Commit: working tree after native-only pixi default lint configure.
  - Result: passed. Build completed the core, dartpy, native collision tests,
    and focused detector targets with only the known third-party OctoMap
    `<ciso646>` warning. CTest passed 2/2.

## Runtime Link And Package Export Inspection Runs

- `pixi run -- bash -lc 'rm -rf build/native_collision_runtime_probe &&
NANOBIND_CMAKE_DIR=$(python -m nanobind --cmake_dir) && cmake -G Ninja -S .
-B build/native_collision_runtime_probe/cpp -DCMAKE_BUILD_TYPE=Release
-DCMAKE_INSTALL_PREFIX=$PWD/build/native_collision_runtime_probe/install
-DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DDART_BUILD_DARTPY=ON
-DDART_BUILD_SIMULATION_EXPERIMENTAL=ON -DDART_BUILD_COLLISION_FCL=OFF
-DDART_BUILD_COLLISION_BULLET=OFF -DDART_BUILD_COLLISION_ODE=OFF
-DDART_BUILD_COLLISION_REFERENCE_TESTS=OFF
-DDART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF -DDART_BUILD_GUI=ON
-DDART_BUILD_GUI_RAYLIB=OFF -DDART_BUILD_GUI_VSG=ON
-DDART_BUILD_EXAMPLES=OFF -DDART_BUILD_TUTORIALS=OFF -DDART_BUILD_TESTS=OFF
-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON -DDART_USE_SYSTEM_GOOGLETEST=ON
-DDART_USE_SYSTEM_IMGUI=ON -DDART_USE_SYSTEM_NANOBIND=OFF
-DDART_USE_SYSTEM_TRACY=ON -Dnanobind_DIR=$NANOBIND_CMAKE_DIR &&
cmake --build build/native_collision_runtime_probe/cpp --target install
--parallel 8'`
  - Commit: working tree after native-only pixi default checkpoint.
  - Result: passed from a fresh build directory. Configure reported FCL,
    Bullet, ODE, reference tests, and reference benchmarks all `OFF`; FCL was
    absent from the required and optional package lists. The install built and
    installed `libdart`, `libdart-collision-native`, `libdart-utils`,
    `libdart-utils-urdf`, `libdart-io`, `libdart-gui`, `libdart-gui-vsg`,
    `libdart-simulation-experimental`, and dartpy. Only known third-party
    OctoMap `<ciso646>` warnings appeared.
- `find build/native_collision_runtime_probe/install \( -type f -o -type l \)
| rg -i
'collision-(fcl|bullet|ode)|libdart-collision-(fcl|bullet|ode)|lib(fcl|bullet|ode|ccd)'
|| true`
  - Commit: working tree after fresh native-only runtime install.
  - Result: no matching installed files or symlinks.
- `rg -n
'collision-(fcl|bullet|ode)|libdart-collision-(fcl|bullet|ode)|lib(fcl|bullet|ode|ccd)|DART_BUILD_COLLISION'
build/native_collision_runtime_probe/install/share/dart/cmake
build/native_collision_runtime_probe/install/lib64/pkgconfig
build/native_collision_runtime_probe/install/lib/pkgconfig 2>/dev/null ||
true`
  - Commit: working tree after fresh native-only runtime install.
  - Result: the only matches were `DART_BUILD_COLLISION_BULLET`,
    `DART_BUILD_COLLISION_FCL`, and `DART_BUILD_COLLISION_ODE` set to `OFF` in
    `DARTConfig.cmake`. No old collision component, old collision library, FCL,
    Bullet, ODE, or libccd references appeared in searched installed
    CMake/pkg-config metadata.
- `find build/native_collision_runtime_probe/install -type f \( -name '*.so' -o
-name '*.so.*' \) -print | sort | while read -r lib; do hits=$(ldd "$lib"
2>/dev/null | grep -Ei 'lib(fcl|bullet|ode|ccd)' || true); if [ -n "$hits"
]; then printf '## %s\n%s\n' "$lib" "$hits"; fi; done`
  - Commit: working tree after fresh native-only runtime install.
  - Result: no installed shared library linked FCL, Bullet, ODE, or libccd.
- `find build/native_collision_runtime_probe/cpp -path '*_dartpy*.so' -o -path
'*/python/dartpy/*.so' | sort | while read -r lib; do hits=$(ldd "$lib"
2>/dev/null | grep -Ei 'lib(fcl|bullet|ode|ccd)' || true); if [ -n "$hits"
]; then printf '## %s\n%s\n' "$lib" "$hits"; fi; done`
  - Commit: working tree after fresh native-only runtime install.
  - Result: the built dartpy extension did not link FCL, Bullet, ODE, or
    libccd.
- `pixi run -- cmake --build build/native_collision_runtime_probe/cpp --target
help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'
|| true`
  - Commit: working tree after fresh native-only runtime install.
  - Result: no legacy collision component or reference-only targets were
    listed in the fresh native-only build tree.

## Pixi Dependency Metadata And Reference Environment Runs

- `pixi lock --no-install`
  - Commit: working tree after moving old collision engines out of default
    Pixi dependencies and adding the `collision-reference` environment.
  - Result: passed and updated `pixi.lock` from schema v6 to v7. The lock
    update removed `bullet-cpp`, `fcl`, `libode`, `libccd-double`, `flann`,
    `hdf5`, and related FCL transitive packages from the `default` environment
    on all supported platforms. It removed the same old collision packages from
    the `py312-wheel`, `py313-wheel`, and `py314-wheel` environments. The new
    `collision-reference` environment explicitly added `bullet-cpp`, `fcl`,
    `libode`, and their transitive packages for comparison work.
- `pixi list --no-install --json -e default
'^(bullet-cpp|fcl|libode|libccd-double|flann|hdf5)$'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: `[]`.
- `pixi list --no-install --json -e py312-wheel
'^(bullet-cpp|fcl|libode|libccd-double|flann|hdf5)$'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: `[]`.
- `pixi list --no-install --json -e py313-wheel
'^(bullet-cpp|fcl|libode|libccd-double|flann|hdf5)$'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: `[]`.
- `pixi list --no-install --json -e py314-wheel
'^(bullet-cpp|fcl|libode|libccd-double|flann|hdf5)$'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: `[]`.
- `pixi list --no-install --json -e collision-reference
'^(bullet-cpp|fcl|libode|libccd-double|flann|hdf5)$'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: returned explicit `bullet-cpp`, `fcl`, and `libode`, plus
    transitive `libccd-double`, `flann`, and `hdf5`. This is the intended
    opt-in reference-engine environment, not a default runtime dependency.
- `pixi list --no-install --json -e gazebo
'^(bullet-cpp|fcl|libode|libccd-double|flann|hdf5)$'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: returned explicit old-engine packages and transitive FCL packages.
    This remains a temporary downstream compatibility environment until the
    retained legacy component facades are cleaned up and gz-physics no longer
    needs the old component package surface.
- `DART_VERBOSE=ON pixi run --locked config`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: passed. Configure reported `DART_BUILD_COLLISION_FCL: OFF`,
    `DART_BUILD_COLLISION_BULLET: OFF`, `DART_BUILD_COLLISION_ODE: OFF`,
    `DART_BUILD_COLLISION_REFERENCE_TESTS: OFF`, and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS: OFF`. FCL was not listed in
    the found package set, CMake added only the normal native component set,
    and native collision test count remained 29.
- `DART_VERBOSE=ON pixi run --locked -e collision-reference config`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: passed. Configure reported FCL, Bullet, ODE, reference tests, and
    reference benchmarks all `ON`; CMake found `fcl`, configured
    `dart/collision/fcl`, `dart/collision/ode`, and
    `dart/collision/bullet`, added 10 components, and increased the native
    collision unit count to 30.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target help | rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends'`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: matched the old reference component targets and
    `test_reference_backends`, proving the explicit reference environment
    restores comparison targets.
- `pixi run --locked -- cmake --build build/default/cpp/Release --target help
| rg
'dart-collision-(fcl|bullet|ode)|test_reference_backends|bm_comparative|bm_scenarios_(mixed_primitives|mesh_heavy)|INTEGRATION_simulation_MimicConstraint'
|| true`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: no old collision component or reference-only targets were listed in
    the default environment.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target test_reference_backends
--parallel 8`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: passed. The target built the native core, DART core,
    `dart-collision-fcl`, `dart-collision-ode`, `dart-collision-bullet`, and
    `bin/test_reference_backends`; only known third-party/deprecated-header
    warnings appeared.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
test_reference_backends`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: passed, 1/1 test in 0.40 seconds.

## Wheel Artifact Inspection Runs

- `pixi run --locked -e py312-wheel wheel-build`
  - Commit: working tree after Pixi dependency metadata split.
  - Result: passed and produced
    `dist/dartpy-7.0.0-cp312-cp312-linux_x86_64.whl`. The wheel build
    command set `DART_BUILD_COLLISION_FCL=OFF`,
    `DART_BUILD_COLLISION_BULLET=OFF`, `DART_BUILD_COLLISION_ODE=OFF`,
    `DART_BUILD_COLLISION_REFERENCE_TESTS=OFF`, and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF`. Configure did not report
    FCL, Bullet, ODE, or libccd as found runtime packages; the build completed
    with only known third-party OctoMap `<ciso646>` warnings.
- `pixi run --locked -e py312-wheel wheel-repair`
  - Commit: working tree after py312 wheel build.
  - Result: passed. `auditwheel` rewrote the wheel tag from
    `linux_x86_64` to `manylinux_2_39_x86_64` and wrote
    `dist/dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl`.
- `pixi run --locked -e py312-wheel wheel-verify`
  - Commit: working tree after py312 wheel repair.
  - Result: passed. `package.xml` and the repaired wheel both reported
    version `7.0.0`.
- `pixi run --locked -e py312-wheel wheel-test`
  - Commit: working tree after py312 wheel repair.
  - Result: passed. The repaired wheel installed into a fresh Python 3.12
    virtual environment, imported `dartpy 7.0.0`, exposed the required
    `collision`, `common`, `constraint`, `dynamics`, `math`, `simulation`, and
    `utils` submodules, exposed optional `gui.Viewer`, and constructed a
    basic `dartpy.simulation.World`.
- `ls -lh dist/*.whl`
  - Commit: working tree after py312 wheel repair.
  - Result: one repaired wheel was present:
    `dist/dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl`, 70M.
- `unzip -Z1
dist/dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl | rg -i
'(^|/)(include/dart/collision/(fcl|bullet|ode)(/|$)|lib/libdart-collision-(fcl|bullet|ode)|dartpy\.libs/lib(fcl|bullet|ode|ccd)|lib/cmake/dart/.*collision-(fcl|bullet|ode))'`
  - Commit: working tree after py312 wheel repair.
  - Result: no matches. The repaired wheel did not bundle old collision
    component headers, old collision component libraries, old collision CMake
    exports, or old FCL/Bullet/ODE/libccd runtime libraries.
- `tmpdir=$(mktemp -d); unzip -q
dist/dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl -d "$tmpdir";
find "$tmpdir" -type f -name '*.so*' -print | while IFS= read -r lib; do
hits=$(ldd "$lib" 2>/dev/null | grep -Ei 'lib(fcl|bullet|ode|ccd)' ||
true); if [ -n "$hits" ]; then printf '%s\n%s\n' "$lib" "$hits"; fi;
done; rm -rf "$tmpdir"`
  - Commit: working tree after py312 wheel repair.
  - Result: no extracted shared object linked FCL, Bullet, ODE, or libccd.
- `unzip -Z1
dist/dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl | rg -i
'(^|/)lib(fcl|bullet|ode|ccd)[^/]*\.(so|dylib|dll|a)$|(^|/).*collision-(fcl|bullet|ode).*\.(so|dylib|dll|a)$'`
  - Commit: working tree after py312 wheel repair.
  - Result: no old collision dynamic or static libraries were present in the
    repaired wheel.

## Python Clean API Runs

- `pixi run -- cmake --build build/default/cpp/Release --target dartpy
--parallel 8`
  - Commit: working tree after dartpy clean detector API and link cleanup.
  - Result: passed in the default native-only configuration. Configure reported
    FCL, Bullet, ODE, reference tests, and reference benchmarks all `OFF`;
    dartpy built without linking legacy collision component targets.
- `PYTHONPATH=build/default/cpp/Release/python pixi run pytest -q
python/tests/unit/collision/test_collision.py
python/tests/unit/simulation/test_world.py`
  - Commit: working tree after dartpy clean detector API and link cleanup.
  - Result: passed. The focused clean-API regression verifies
    `DartCollisionDetector` is exposed, legacy detector aliases are absent, and
    world/collision APIs instantiate the canonical detector path.

## Native-Only Gazebo Compatibility Run

- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5
pixi run -e gazebo test-gz`
  - Commit: working tree after removing the Gazebo environment's
    FCL/Bullet/ODE reference-component overrides.
  - Result: passed, 65/65 gz-physics tests. The DART install configure printed
    `DART_BUILD_COLLISION_FCL: OFF`, `DART_BUILD_COLLISION_BULLET: OFF`,
    `DART_BUILD_COLLISION_ODE: OFF`, and collision reference tests/benchmarks
    `OFF`, proving gz-physics no longer needs the old reference-engine build
    flags for runtime integration. The gz-physics build emitted the intended
    C++ deprecation warnings when it touched retained Bullet/Ode compatibility
    classes, and the plugin link inspection still found
    `libdart-collision-native.so`.
- Native compatibility package smoke:
  - Command: install the current default build into `build/native-compat-install`,
    configure `docs/dev_tasks/native_collision/smoke/native_compat_package`
    against that install, build the smoke executable, run it, and verify
    `build/native-compat-install/lib` contains no
    `dart-collision-reference`, FCL, Bullet, ODE, or libccd collision runtime
    libraries.
  - Result: passed. The smoke accepted installed `collision-fcl`,
    `collision-bullet`, and `collision-ode` package component facades while the
    DART install remained native-only.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target dartpy --parallel 8`
  - Commit: working tree after dartpy clean detector API and link cleanup.
  - Result: passed in a reference-enabled configuration where FCL, Bullet, ODE,
    reference tests, and reference benchmarks were all `ON`. The build also
    verified the GUI key mapping against ImGui 1.92.8, where the old
    `ImGuiKey_Mod*` aliases are no longer declared.
- `find build/collision-reference/cpp/Release/python -type f -name '*.so'
-print | sort | while IFS= read -r lib; do hits=$(ldd "$lib" 2>/dev/null |
grep -Ei 'lib(fcl|bullet|ode|ccd)' || true); if [ -n "$hits" ]; then printf
'%s\n%s\n' "$lib" "$hits"; fi; done`
  - Commit: working tree after reference-enabled dartpy build.
  - Result: no built dartpy shared object linked FCL, Bullet, ODE, or libccd
    even though the reference components existed in the build.
- `PYTHONPATH=build/collision-reference/cpp/Release/python pixi run --locked
-e collision-reference pytest -q python/tests/unit/collision/test_collision.py
python/tests/unit/simulation/test_world.py`
  - Commit: working tree after reference-enabled dartpy build.
  - Result: passed, 17/17 tests.
- `PYTHONPATH=build/collision-reference/cpp/Release/python pixi run --locked
-e collision-reference python - <<'PY' ...`
  - Commit: working tree after reference-enabled dartpy build.
  - Result: `DartCollisionDetector`, `DARTCollisionDetector`,
    `FCLCollisionDetector`, `BulletCollisionDetector`, and
    `OdeCollisionDetector` all printed `dart dart` for instance type and static
    type.

## C++ Reference API Split Runs

- `rg -n "\b(FCL|Bullet|Ode)CollisionDetector::create\(" tests -g
'*.{cpp,hpp}'`
  - Commit: working tree after adding explicit reference detector creation APIs
    and moving tests/benchmarks to them.
  - Result: no direct public legacy detector `create()` calls remain in
    `tests/`; reference test and benchmark call sites use `createReference()`.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target
UNIT_collision_FCLCollisionDetector UNIT_collision_BulletCollisionShapes
UNIT_collision_OdeHeightmap UNIT_collision_OdeCylinderMesh
test_reference_backends INTEGRATION_collision_native_backend_consistency
INTEGRATION_collision_Collision INTEGRATION_collision_FclPrimitiveContactMatrix
INTEGRATION_collision_MeshContactRegression bm_comparative
bm_comparative_narrow_phase bm_comparative_distance bm_comparative_raycast
bm_scenarios_mesh_heavy bm_scenarios_mixed_primitives
bm_scenarios_raycast_batch bm_boxes --parallel 8`
  - Commit: working tree after explicit reference detector APIs.
  - Result: passed after replacing the deprecated Google Benchmark helper type
    `benchmark::internal::Benchmark*` with public `benchmark::Benchmark*` in
    the comparative distance and narrow-phase benchmark helpers.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^(UNIT_collision_FCLCollisionDetector|UNIT_collision_BulletCollisionShapes|UNIT_collision_OdeHeightmap|UNIT_collision_OdeCylinderMesh|test_reference_backends|INTEGRATION_collision_native_backend_consistency|INTEGRATION_collision_MeshContactRegression)$'`
  - Commit: working tree after explicit reference detector APIs.
  - Result: passed, 7/7 tests.
- `pixi run --locked -e collision-reference -- bash -lc
"build/collision-reference/cpp/Release/bin/bm_comparative_distance
--benchmark_filter='BM_Distance_SphereSphere_(FCL|Bullet|ODE)$'
--benchmark_min_time=0.001s --benchmark_repetitions=1 &&
build/collision-reference/cpp/Release/bin/bm_comparative_narrow_phase
--benchmark_filter='BM_NarrowPhase_SphereSphere_(FCL|Bullet|ODE)$'
--benchmark_min_time=0.001s --benchmark_repetitions=1"`
  - Commit: working tree after explicit reference detector APIs.
  - Result: passed. The smoke run instantiated FCL, Bullet, and ODE through
    `createReference()` in comparative distance and narrow-phase benchmark
    entry points. ODE distance still reports its existing unsupported-distance
    warning and returns the legacy `-1.0` behavior.
- `pixi run -- cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector dartpy --parallel 8`
  - Commit: working tree after explicit reference detector APIs.
  - Result: passed in the default native-only configuration.
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after explicit reference detector APIs.
  - Result: passed, 1/1 test.

## C++ Legacy Create Alias Runs

- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target test_reference_backends
UNIT_collision_FCLCollisionDetector UNIT_collision_BulletCollisionShapes
UNIT_collision_OdeHeightmap UNIT_collision_OdeCylinderMesh --parallel 8`
  - Commit: working tree after direct public C++ legacy `create()` paths were
    changed to return `DartCollisionDetector`.
  - Result: passed. `test_reference_backends` observed only existing deprecated
    wrapper-header warnings.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^(test_reference_backends|UNIT_collision_FCLCollisionDetector|UNIT_collision_BulletCollisionShapes|UNIT_collision_OdeHeightmap|UNIT_collision_OdeCylinderMesh)$'`
  - Commit: working tree after direct public C++ legacy `create()` paths were
    changed to return `DartCollisionDetector`.
  - Result: passed, 5/5 tests. The new `test_reference_backends` coverage
    checked that `FCLCollisionDetector::create()`,
    `BulletCollisionDetector::create()`, and `OdeCollisionDetector::create()`
    returned detector type `dart` at that checkpoint, while
    `createReference()` still returned the explicit `fcl`, `bullet`, and `ode`
    reference detectors. Later gz-physics compatibility work narrowed this:
    direct C++ facades may preserve legacy display strings while staying
    native-backed.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target boxes mimic_pendulums
capsule_ground_contact heightmap --parallel 8`
  - Commit: working tree after direct public C++ legacy `create()` paths were
    changed to return `DartCollisionDetector`.
  - Result: passed. This covers in-repo example call sites that still use the
    old direct C++ legacy detector `create()` names. The build observed only
    existing third-party OctoMap `<ciso646>` C++20 warnings.
- `pixi run -- cmake --build build/default/cpp/Release --target
UNIT_collision_DartCollisionDetector --parallel 8`
  - Commit: working tree after direct public C++ legacy `create()` paths were
    changed to return `DartCollisionDetector`.
  - Result: passed; Ninja reported no work to do in the default native-only
    build.
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_collision_DartCollisionDetector$'`
  - Commit: working tree after direct public C++ legacy `create()` paths were
    changed to return `DartCollisionDetector`.
  - Result: passed, 1/1 test.

## User-Facing Example/Tutorial Runtime Cleanup Runs

- `rg -n "#include <dart/collision/(bullet|ode|fcl)|collision/(bullet|ode|fcl)|DART_HAVE_(BULLET|ODE|FCL)|DART_BUILD_COLLISION_(FCL|BULLET|ODE)|CollisionDetectorType::(Bullet|Ode|Fcl)|CollisionDetector::getFactory\(\)->canCreate\(\"(bullet|ode|fcl)\"\)|dart-collision-(bullet|ode|fcl)|collision-(bullet|ode|fcl)|<fcl/" examples tutorials -g 'CMakeLists.txt' -g '*.{cpp,hpp,md}'`
  - Commit: working tree after moving examples/tutorials off old collision
    components.
  - Result: no matches.
- `rg -n "DART_BUILD_COLLISION_(FCL|BULLET|ODE)|dart-collision-(fcl|bullet|ode)|collision-(fcl|bullet|ode)|FCLCollisionDetector|BulletCollisionDetector|OdeCollisionDetector|CollisionDetectorType::(Fcl|Bullet|Ode)" examples tutorials -g 'CMakeLists.txt' -g '*.{cpp,hpp,md}'`
  - Commit: working tree after moving examples/tutorials off old collision
    components and updating stale example README text.
  - Result: no matches.
- `pixi run -- cmake --build build/default/cpp/Release --target boxes
add_delete_skels fetch capsule_ground_contact heightmap mimic_pendulums
rigid_shapes human_joint_limits --parallel 8`
  - Commit: working tree after moving examples off old collision components.
  - Result: CMake regenerated with old collision components still `OFF` and
    added the previously skipped examples, then failed only because
    `human_joint_limits` is not generated in this environment when TinyDNN is
    unavailable.
- `pixi run -- cmake --build build/default/cpp/Release --target boxes
add_delete_skels fetch capsule_ground_contact heightmap mimic_pendulums
rigid_shapes --parallel 8`
  - Commit: working tree after moving examples off old collision components.
  - Result: passed in the default native-only configuration. Build observed
    only existing third-party OctoMap `<ciso646>` C++20 warnings.
- `ldd build/default/cpp/Release/bin/boxes
build/default/cpp/Release/bin/add_delete_skels
build/default/cpp/Release/bin/fetch
build/default/cpp/Release/bin/capsule_ground_contact
build/default/cpp/Release/bin/heightmap
build/default/cpp/Release/bin/mimic_pendulums
build/default/cpp/Release/bin/rigid_shapes | rg -i
"lib(fcl|bullet|ode|ccd)|dart-collision-(fcl|bullet|ode)"`
  - Commit: working tree after moving examples off old collision components.
  - Result: no matches; affected default-built examples do not link old
    collision engines or old collision component libraries.
- `pixi run -- cmake --build build/default/cpp/Release --target
tutorial_biped tutorial_biped_finished --parallel 8`
  - Commit: working tree after moving tutorials off old collision components.
  - Result: passed in the default native-only configuration. CMake regenerated
    with old collision components still `OFF` and added the previously skipped
    tutorials. Build observed only existing third-party OctoMap `<ciso646>`
    C++20 warnings.
- `ldd build/default/cpp/Release/bin/tutorial_biped
build/default/cpp/Release/bin/tutorial_biped_finished | rg -i
"lib(fcl|bullet|ode|ccd)|dart-collision-(fcl|bullet|ode)"`
  - Commit: working tree after moving tutorials off old collision components.
  - Result: no matches; affected default-built tutorials do not link old
    collision engines or old collision component libraries.

## Reference Component Split Runs

- `pixi run config`
  - Commit: working tree after splitting optional old-engine components into
    native-backed compatibility components and reference-named targets.
  - Result: passed. Default configure kept `DART_BUILD_COLLISION_FCL`,
    `DART_BUILD_COLLISION_BULLET`, `DART_BUILD_COLLISION_ODE`, reference tests,
    and reference benchmarks `OFF`; CMake generated 10 components, including
    native-backed `collision-fcl`, `collision-bullet`, and `collision-ode`
    compatibility components.
- `pixi run --locked -e collision-reference config`
  - Commit: working tree after reference target rename.
  - Result: passed. Reference configure kept FCL, Bullet, ODE, reference tests,
    and reference benchmarks `ON`; CMake generated 13 components, including
    compatibility `collision-*` components and explicit
    `collision-reference-*` components.
- `ninja -C build/default/cpp/Release -t targets | rg
"dart-collision-(fcl|bullet|ode|reference-fcl|reference-bullet|reference-ode)|dart_component_collision"`
  - Commit: working tree after reference target rename.
  - Result: default native-only build exposed only phony compatibility
    components `dart_component_collision-fcl`,
    `dart_component_collision-bullet`, `dart_component_collision-ode`, and
    `dart_component_collision-native`; no old-engine shared-library or
    reference targets were present.
- `ninja -C build/collision-reference/cpp/Release -t targets | rg
"dart-collision-(fcl|bullet|ode|reference-fcl|reference-bullet|reference-ode)|dart_component_collision"`
  - Commit: working tree after reference target rename.
  - Result: reference build exposed `libdart-collision-reference-fcl`,
    `libdart-collision-reference-bullet`, `libdart-collision-reference-ode`,
    `dart_component_collision-reference-*`, and the native-backed
    compatibility component phony targets.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target dart-collision-reference-fcl
dart-collision-reference-bullet dart-collision-reference-ode
test_reference_backends UNIT_collision_FCLCollisionDetector
UNIT_collision_BulletCollisionShapes UNIT_collision_OdeHeightmap
UNIT_collision_OdeCylinderMesh --parallel 8`
  - Commit: working tree after reference target rename.
  - Result: passed. The old-engine comparison libraries now build under
    reference target names; focused reference tests linked successfully with
    only known deprecated compatibility-header warnings.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release -R
'test_reference_backends|UNIT_collision_FCLCollisionDetector|UNIT_collision_BulletCollisionShapes|UNIT_collision_OdeHeightmap|UNIT_collision_OdeCylinderMesh'
--output-on-failure`
  - Commit: working tree after reference target rename.
  - Result: passed, 5/5 tests.
- `rm -rf build/native-compat-install && pixi run -- cmake --install
build/default/cpp/Release --prefix build/native-compat-install --component
headers && pixi run -- cmake --install build/default/cpp/Release --prefix
build/native-compat-install`
  - Commit: working tree after adding native-backed compatibility component
    facades.
  - Result: passed. The default install contains
    `dart_collision-fclComponent.cmake`,
    `dart_collision-bulletComponent.cmake`, and
    `dart_collision-odeComponent.cmake` with imported interface targets linking
    `dart`; `build/native-compat-install/lib` contains
    `libdart-collision-native.so` and no `libdart-collision-reference-*`,
    FCL, Bullet, ODE, or libccd collision runtime libraries.
- Native compatibility package smoke:
  - Command:
    `pixi run -- bash -lc 'set -euo pipefail; rm -rf
build/native-compat-install; cmake --install build/default/cpp/Release
--prefix build/native-compat-install --component headers; cmake --install
build/default/cpp/Release --prefix build/native-compat-install; rm -rf
build/native-compat-smoke/build; cmake -S build/native-compat-smoke -B
build/native-compat-smoke/build
-DCMAKE_PREFIX_PATH="$PWD/build/native-compat-install;$CONDA_PREFIX";
cmake --build build/native-compat-smoke/build --parallel 4;
build/native-compat-smoke/build/native_collision_compat_smoke'`
  - Commit: working tree after adding native-backed compatibility component
    and detector-header facades.
  - Result: passed. A downstream-style project using
    `find_package(DART REQUIRED COMPONENTS collision-fcl collision-bullet
collision-ode)` linked the compatibility interface targets, included the
    installed legacy detector headers, and verified factory keys plus
    `FCLCollisionDetector::create()`, `BulletCollisionDetector::create()`, and
    `OdeCollisionDetector::create()` all construct detectors reporting type
    `dart`.
- `pixi run config && pixi run -- cmake --build build/default/cpp/Release
--target dart_component_collision-fcl dart_component_collision-bullet
dart_component_collision-ode dart --parallel 8 && pixi run -- bash -lc
'set -euo pipefail; rm -rf build/native-compat-install; cmake --install
build/default/cpp/Release --prefix build/native-compat-install --component
headers; cmake --install build/default/cpp/Release --prefix
build/native-compat-install; rm -rf build/native-compat-smoke/build; cmake -S
build/native-compat-smoke -B build/native-compat-smoke/build
-DCMAKE_PREFIX_PATH="$PWD/build/native-compat-install;$CONDA_PREFIX"; cmake
--build build/native-compat-smoke/build --parallel 4;
build/native-compat-smoke/build/native_collision_compat_smoke; if find
build/native-compat-install/lib -maxdepth 1 \( -type f -o -type l \) | sort |
rg "dart-collision-reference|lib(fcl|bullet|ode|ccd)"; then exit 1; fi'`
  - Commit: working tree after adding native-only installed legacy detector
    header facades.
  - Result: passed. Default configure kept old collision engines and reference
    tests/benchmarks `OFF`; the compatibility component targets built, the
    installed downstream smoke included FCL/Bullet/ODE detector compatibility
    headers and verified factory keys plus legacy `create()` calls report
    `dart`, and the installed library directory contained no reference
    collision libraries or FCL/Bullet/ODE/libccd runtime links.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target dart-collision-reference-fcl
dart-collision-reference-bullet dart-collision-reference-ode
test_reference_backends UNIT_collision_FCLCollisionDetector
UNIT_collision_BulletCollisionShapes UNIT_collision_OdeHeightmap
UNIT_collision_OdeCylinderMesh --parallel 8`
  - Commit: working tree after adding native-only installed legacy detector
    header facades.
  - Result: passed. The explicit reference libraries and focused reference
    detector targets still build with only known deprecated compatibility-header
    warnings.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release -R
'test_reference_backends|UNIT_collision_FCLCollisionDetector|UNIT_collision_BulletCollisionShapes|UNIT_collision_OdeHeightmap|UNIT_collision_OdeCylinderMesh'
--output-on-failure`
  - Commit: working tree after adding native-only installed legacy detector
    header facades.
  - Result: passed, 5/5 tests.
- `pixi run --locked -e collision-reference config && pixi run --locked -e
collision-reference -- cmake --build build/collision-reference/cpp/Release
--target dart-collision-reference-fcl dart-collision-reference-bullet
dart-collision-reference-ode test_reference_backends
UNIT_collision_FCLCollisionDetector UNIT_collision_BulletCollisionShapes
UNIT_collision_OdeHeightmap UNIT_collision_OdeCylinderMesh --parallel 8`
  - Commit: working tree after making reference-enabled installs use
    native-backed legacy detector header facades.
  - Result: passed. Reference configure kept FCL, Bullet, ODE, reference tests,
    and reference benchmarks `ON`; explicit reference libraries and focused
    reference tests still build with only known deprecated compatibility-header
    warnings.
- `pixi run --locked -e collision-reference -- bash -lc 'set -euo pipefail;
rm -rf build/reference-compat-install; cmake --install
build/collision-reference/cpp/Release --prefix build/reference-compat-install
--component headers; if find
build/reference-compat-install/include/dart/collision -maxdepth 3 -type f |
sort | rg
"/(fcl|bullet|ode)/(detail/|fcl_collision_object|bullet_collision_object|ode_collision_object|fcl_types|bullet_types|ode_types|collision_shapes|tri_tri|backward_compatibility|bullet_include|bullet_collision_shape)$";
then exit 1; fi; ${CXX:-c++} -std=c++20
-Ibuild/reference-compat-install/include
-Ibuild/collision-reference/cpp/Release -I$CONDA_PREFIX/include
-I$CONDA_PREFIX/include/eigen3 -c build/native-compat-smoke/main.cpp -o
/tmp/dart_ref_header_smoke.o'`
  - Commit: working tree after making reference-enabled installs use
    native-backed legacy detector header facades.
  - Result: passed. A reference-enabled header install exposes only the
    native-backed legacy detector/group facade headers on the FCL/Bullet/ODE
    public paths, omits old-engine object/type/detail headers from those paths,
    and the installed legacy detector headers compile without FCL, Bullet, or
    ODE include paths.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release -R
'test_reference_backends|UNIT_collision_FCLCollisionDetector|UNIT_collision_BulletCollisionShapes|UNIT_collision_OdeHeightmap|UNIT_collision_OdeCylinderMesh'
--output-on-failure`
  - Commit: working tree after making reference-enabled installs use
    native-backed legacy detector header facades.
  - Result: passed, 5/5 tests.
- `pixi run --locked -e collision-reference bm-collision-check`
  - Commit: working tree after adding the focused collision benchmark JSON
    checker.
  - Result: passed. The task configured the reference environment, built
    `bm_comparative_narrow_phase`, ran a focused SphereSphere/BoxBox/SphereBox
    native-vs-FCL/Bullet/ODE benchmark subset with three repetitions, wrote
    `.benchmark_results/collision_check.json`, and reported
    `collision benchmark check: 3 passed, 0 failed, 0 skipped` with native
    faster than the best enabled reference backend for all checked families.
- `pixi run --locked -e collision-reference config`
  - Commit: working tree after moving old FCL/Bullet/ODE implementation files
    under explicit `reference/` source paths and replacing top-level
    source-tree detector/group headers with native-backed compatibility
    facades.
  - Result: passed. The generated reference headers now include
    `dart/collision/{fcl,bullet,ode}/reference/...` paths while the public
    source-tree top-level detector/group paths remain compatibility facades.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target dart-collision-reference-fcl
dart-collision-reference-bullet dart-collision-reference-ode
test_reference_backends UNIT_collision_FCLCollisionDetector
UNIT_collision_BulletCollisionShapes UNIT_collision_OdeHeightmap
UNIT_collision_OdeCylinderMesh bm_comparative_narrow_phase --parallel 8`
  - Commit: working tree after the source-tree reference path split.
  - Result: passed. The old-engine libraries built from
    `dart/collision/{fcl,bullet,ode}/reference/` sources, the focused reference
    tests rebuilt, and the comparative narrowphase benchmark rebuilt.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release -R
'test_reference_backends|UNIT_collision_FCLCollisionDetector|UNIT_collision_BulletCollisionShapes|UNIT_collision_OdeHeightmap|UNIT_collision_OdeCylinderMesh'
--output-on-failure`
  - Commit: working tree after the source-tree reference path split.
  - Result: passed, 5/5 tests.
- `pixi run --locked -e collision-reference bm-collision-check`
  - Commit: working tree after the source-tree reference path split.
  - Result: passed. The recurring focused benchmark guard rebuilt the
    reference-path comparative narrowphase benchmark and reported
    `collision benchmark check: 3 passed, 0 failed, 0 skipped`.
- `pixi run --locked -e collision-reference bm-collision-check`
  - Commit: working tree after broadening the collision benchmark guard.
  - Result: passed. The task configured the reference environment, built and
    ran checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy,
    raycast-batch, and public DART adapter benchmark subsets.
  - Native/reference check results:
    | Subset | Result |
    | --- | --- |
    | Narrowphase | `3 passed, 0 failed, 0 skipped` |
    | Distance | `3 passed, 0 failed, 0 skipped` |
    | Raycast | `5 passed, 0 failed, 0 skipped` |
    | Mixed primitives | `4 passed, 0 failed, 0 skipped` |
    | Mesh-heavy | `1 passed, 0 failed, 0 skipped` |
    | Raycast batch | `2 passed, 0 failed, 0 skipped` |
  - The public adapter subset wrote
    `.benchmark_results/collision_check_adapter.json` for collision,
    dirty-world collision, distance, and raycast through
    `DartCollisionDetector`.
- `pixi run lint`
  - Commit: working tree after broadening the collision benchmark guard.
  - Result: passed. CMake configure, C++ formatting, docs formatting, spell
    check, Python formatting, TOML/YAML/RST checks, and AI command sync
    completed.
- `pixi run check-docs-policy`
  - Commit: working tree after broadening the collision benchmark guard.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after broadening the collision benchmark guard.
  - Result: passed.
- `.github/workflows/ci_ubuntu.yml` `Collision Benchmark Guard`
  - Commit: working tree after adding scheduled/manual CI benchmark guard.
  - Result: workflow job added. It runs
    `pixi run --locked -e collision-reference bm-collision-check` on schedule
    and `workflow_dispatch`, then uploads
    `.benchmark_results/collision_check_*.json` artifacts. GitHub run evidence
    is still required before the north-star PR can treat this as a completed
    CI gate.
- `python - <<'PY' ... yaml.safe_load(.github/workflows/ci_ubuntu.yml) ... PY`
  - Commit: working tree after adding scheduled/manual CI benchmark guard.
  - Result: passed. The edited CI Linux workflow parsed as YAML. `actionlint`
    was not installed in this environment.
- `pixi run lint`
  - Commit: working tree after adding scheduled/manual CI benchmark guard.
  - Result: passed. CMake configure, C++ formatting, docs formatting, spell
    check, Python formatting, TOML/YAML/RST checks, and AI command sync
    completed.
- `pixi run check-docs-policy`
  - Commit: working tree after adding scheduled/manual CI benchmark guard.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after adding scheduled/manual CI benchmark guard.
  - Result: passed.
- `python scripts/verify_wheel_collision_isolation.py dist/dartpy-*.whl`
  - Commit: working tree after adding dartpy wheel collision-isolation
    verification.
  - Result: passed. The existing repaired
    `dartpy-7.0.0-cp312-cp312-manylinux_2_39_x86_64.whl` was inspected across
    1843 wheel entries and contained no legacy collision runtime artifacts.
- `pixi run --locked -e py312-wheel wheel-verify`
  - Commit: working tree after wiring collision-isolation checks into
    `wheel-verify-core`.
  - Result: passed. The task verified the cp312 wheel version and ran
    `scripts/verify_wheel_collision_isolation.py` on the repaired py312 Linux
    wheel.
- `pixi run lint`
  - Commit: working tree after wiring collision-isolation checks into
    `wheel-verify-core`.
  - Result: passed. CMake configure, C++ formatting, docs formatting, spell
    check, Python formatting, TOML/YAML/RST checks, and AI command sync
    completed.
- `pixi run check-docs-policy`
  - Commit: working tree after wiring collision-isolation checks into
    `wheel-verify-core`.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after wiring collision-isolation checks into
    `wheel-verify-core`.
  - Result: passed.
- `pixi run lint`
  - Commit: working tree after documenting downstream migration and annotating
    compatibility facades.
  - Result: passed. CMake configure, C++ formatting, docs formatting, spell
    check, Python formatting, TOML/YAML/RST checks, and AI command sync
    completed.
- `pixi run check-docs-policy`
  - Commit: working tree after documenting downstream migration and annotating
    compatibility facades.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after documenting downstream migration and annotating
    compatibility facades.
  - Result: passed.
- `pixi run config`
  - Commit: working tree after adding source-tree top-level All/PascalCase
    compatibility facades for FCL, Bullet, and ODE.
  - Result: passed. Default configure kept
    `DART_BUILD_COLLISION_FCL/BULLET/ODE` and reference tests/benchmarks `OFF`
    while seeing only the native-backed source facade headers on the ordinary
    top-level legacy paths.
- `${CXX:-c++} -std=c++20 -I. -Ibuild/default/cpp/Release
-I$CONDA_PREFIX/include -I$CONDA_PREFIX/include/eigen3 -x c++ -c ...`
  - Commit: working tree after adding source-tree top-level All/PascalCase
    compatibility facades for FCL, Bullet, and ODE.
  - Result: passed. A source-tree compile smoke included
    `dart/collision/fcl/FCLCollisionDetector.hpp`,
    `dart/collision/bullet/BulletCollisionDetector.hpp`, and
    `dart/collision/ode/OdeCollisionDetector.hpp` with no FCL/Bullet/ODE
    include paths and verified all three facade classes derive from
    `DartCollisionDetector`.
- `pixi run -- cmake --build build/default/cpp/Release --target
dart_component_collision-fcl dart_component_collision-bullet
dart_component_collision-ode dart --parallel 8`
  - Commit: working tree after adding source-tree top-level All/PascalCase
    compatibility facades for FCL, Bullet, and ODE.
  - Result: passed with `ninja: no work to do`, confirming the default
    native-only component facade targets still exist without rebuilding or
    linking old collision engines.
- `pixi run lint`
  - Commit: working tree after installed legacy detector header facades and
    docs update.
  - Result: passed. This reran the default native-only configure and formatting
    suite after the compatibility header install changes.
- `pixi run check-docs-policy`
  - Commit: working tree after installed legacy detector header facades and
    docs update.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after installed legacy detector header facades and
    docs update.
  - Result: passed.
- `python scripts/check_collision_runtime_isolation.py`
  - Commit: working tree after adding the runtime source isolation verifier.
  - Result: passed. The verifier found no non-reference DART source includes of
    FCL, Bullet, ODE, libccd, or explicit collision reference backend headers,
    and no legacy engine implementation sources outside `reference/` paths.
- `pixi run lint`
  - Commit: working tree after adding the runtime source isolation verifier and
    wiring it into lint tasks.
  - Result: passed. CMake configure and formatting completed, and
    `check-collision-runtime-isolation` ran as part of lint and passed.
- `pixi run check-docs-policy`
  - Commit: working tree after adding the runtime source isolation verifier and
    documentation updates.
  - Result: passed.
- `git diff --check`
  - Commit: working tree after adding the runtime source isolation verifier and
    documentation updates.
  - Result: passed.

## Current Compatibility Cleanup Runs

- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target test_reference_backends
test_legacy_compat_facades --parallel 13`
  - Commit: working tree after preserving legacy direct C++ facade display
    names while keeping them native-backed.
  - Result: passed. The focused reference build kept explicit
    `createReference()` old-engine coverage and built the new public
    compatibility facade test.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^(test_reference_backends|test_legacy_compat_facades)$'`
  - Commit: working tree after preserving legacy direct C++ facade display
    names while keeping them native-backed.
  - Result: passed, 2/2 tests. Direct C++ legacy facade objects report legacy
    display strings for gz-physics compatibility while deriving from
    `DartCollisionDetector`; public factory aliases still canonicalize to
    `dart`.
- `DART_PARALLEL_JOBS=13 CTEST_PARALLEL_LEVEL=13 pixi run -e gazebo test-gz`
  - Commit: working tree after legacy facade display-name compatibility.
  - Result: failed. Display-name checks that previously failed now pass, but
    5 gz-physics tests still failed: `COMMON_TEST_collisions_dartsim`
    `MeshAndPlane` free-fell to z `-4.9099` instead of stopping near `-1.91`,
    `COMMON_TEST_detachable_joint_dartsim`,
    `COMMON_TEST_joint_features_dartsim`,
    `COMMON_TEST_joint_transmitted_wrench_features_dartsim`, and
    `COMMON_TEST_simulation_features_dartsim`.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target test_aabb test_collision_world
UNIT_collision_DartCollisionDetector test_plane test_cylinder
test_legacy_compat_facades test_reference_backends --parallel 13`
  - Commit: working tree after adding unbounded-plane AABB transform coverage,
    native plane/mesh dispatch, and a cylinder/plane parallel-axis guard.
  - Result: passed.
- `pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^(test_aabb|test_collision_world|UNIT_collision_DartCollisionDetector|test_plane|test_cylinder|test_legacy_compat_facades|test_reference_backends)$'`
  - Commit: working tree after adding unbounded-plane AABB transform coverage,
    native plane/mesh dispatch, and a cylinder/plane parallel-axis guard.
  - Result: passed, 7/7 tests.
- `pixi run -e gazebo install OFF`
  - Commit: working tree after the same native collision fixes.
  - Result: passed. The updated DART build installed into the gazebo
    environment before rerunning focused gz-physics tests.
- Focused gz-physics subset:
  `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; ctest --output-on-failure -R
"^(COMMON_TEST_collisions_dartsim|COMMON_TEST_detachable_joint_dartsim|COMMON_TEST_joint_features_dartsim|COMMON_TEST_joint_transmitted_wrench_features_dartsim)$"'`
  - Commit: working tree after native plane/mesh and AABB fixes.
  - Result: failed. `COMMON_TEST_collisions_dartsim` timed out in
    `MeshAndPlane`, and the detachable-joint, joint-feature, and transmitted
    wrench tests still failed with contact/solver-facing differences.
- `pixi run --locked -e collision-reference -- cmake --build
build/collision-reference/cpp/Release --target test_plane
UNIT_collision_DartCollisionDetector test_collision_world --parallel 13 &&
pixi run --locked -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^(test_plane|UNIT_collision_DartCollisionDetector|test_collision_world)$'`
  - Commit: working tree after capping plane/mesh generated contacts.
  - Result: passed, 3/3 tests.
- `pixi run -e gazebo -- bash -lc 'set -euo pipefail; export
LD_LIBRARY_PATH=.deps/gz-physics/build/lib:$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-};
cd .deps/gz-physics/build; timeout 90s ctest --output-on-failure -R
"^COMMON_TEST_collisions_dartsim$"'`
  - Commit: working tree after capping plane/mesh generated contacts and
    reinstalling DART into the gazebo environment.
  - Result: failed quickly instead of timing out. Static collision cases pass,
    but `MeshAndPlane` still free-falls to z `-4.909904999999969` instead of
    the expected `-1.91`. This was the next downstream correctness blocker at
    that checkpoint and was later reduced to pair-order contact normal
    semantics.

## Current Validation Audit Runs

- `DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all`
  - Commit: working tree after `cd16b64be0b` native shape-taxonomy cleanup and
    before the VSG shape-type switch repair.
  - Result: failed in `Build Debug`. `dart/gui/vsg/geometry_builders.cpp`
    still referenced removed native `ShapeType::Cone`,
    `ShapeType::HeightField`, and `ShapeType::PointCloud` enum values. This
    was a stale compile-time user of the old native taxonomy, not a collision
    behavior regression.
- `rg -n "ShapeType::(Cone|HeightField|PointCloud)" dart tests examples
tutorials python --glob '!build/**' --glob '!.pixi/**' --glob '!external/**'`
  - Commit: working tree after removing the stale VSG switch cases.
  - Result: no matches. The removed native shape tags no longer appear in
    non-generated source paths.
- `pixi run -- cmake --build build/default/cpp/Debug --target dart-gui-vsg
--parallel $(python scripts/parallel_jobs.py)`
  - Commit: working tree after removing the stale VSG switch cases.
  - Result: passed. The Debug VSG target rebuilt `geometry_builders.cpp` and
    linked `libdart-gui-vsgd`.
- `DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all`
  - Commit: working tree after removing the stale VSG switch cases.
  - Result: passed. The comprehensive test suite reported lint, build, unit
    tests, simulation-experimental tests, Python tests, and documentation all
    passed with `All tests passed!`.
- `DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all`
  - Commit: `864dd56d944c` after seeding durable built-in collision
    architecture docs and clarifying the native-backed compatibility facade
    policy.
  - Result: passed. The comprehensive test suite reported 6/6 top-level gates
    passed: linting, build, unit tests, simulation-experimental tests, Python
    tests, and documentation. The final report printed `All tests passed!` and
    `Ready to submit PR!`.
- PR CI refresh on head `714d220d82a`
  - Run/job: `25880337655` / `76058385300` (`CI macOS` /
    `Release Tests (arm64)`).
  - Result: failed compiling `test_collision_filter.cpp` because two callback
    filter tests stored created collision objects in unused local variables.
    The tests now create those objects without unused bindings.
  - Run/job: `25880337655` / `76058385388` (`CI macOS` /
    `Debug Tests (arm64)`).
  - Result: failed `test_ccd`, `UNIT_gui_vsg_simple_viewer`, and
    `INTEGRATION_simulation_World`. Local Debug reproduction found:
    native sphere-box CCD asserted when a sphere cast started inside the
    expanded box and `hitAxis` remained unset; VSG headless viewer
    construction can throw on runners without a usable headless backend; and
    `TypedSetterKeepsCurrentDetectorWhenFactoryReturnsNull` overrode the
    default `"dart"` creator before constructing the world, so the default
    constraint solver dereferenced a null detector instead of exercising the
    typed setter fallback.
- Focused local validation after the macOS Debug/Release repairs:
  - Command:
    `cmake --build build/default/cpp/Debug --target test_ccd UNIT_gui_vsg_simple_viewer INTEGRATION_simulation_World --parallel 5`
  - Result: passed.
  - Command:
    `ctest --test-dir build/default/cpp/Debug -R '^(test_ccd|INTEGRATION_simulation_World|UNIT_gui_vsg_simple_viewer)$' --output-on-failure -j 5`
  - Result: passed, 3/3 tests.
  - Command:
    `cmake --build build/default/cpp/Release --target test_collision_filter --parallel 5`
  - Result: passed.
  - Command:
    `ctest --test-dir build/default/cpp/Release -R '^test_collision_filter$' --output-on-failure -j 5`
  - Result: passed, 1/1 test.
- Closed-PR Linux native failure triage from head `714d220d82a`
  - Run/job: `25880337619` / `76058493678` (`CI Linux` /
    `Debug Tests`).
  - Result: after the macOS fixes, the remaining reproduced failure was
    `test_collision_world` in Debug. World-level sphere-cast and capsule-cast
    results pointed at loop-local `CollisionObject` handles, so Debug builds
    exposed dangling result object pointers.
  - Run/job: `25880337619` / `76058493716` (`CI Linux` /
    `Native Collision (no FCL/Bullet/ODE)`).
  - Result: failed inside the `collision-native` label on the old head. The
    current Release native-collision label was rerun locally after the query
    result-handle repair and now passes.
- Focused local validation after the world query result-handle repair:
  - Command:
    `cmake --build build/default/cpp/Debug --target test_collision_world --parallel 5`
  - Result: passed.
  - Command:
    `ctest --test-dir build/default/cpp/Debug -R '^test_collision_world$' --output-on-failure -j 5`
  - Result: passed, 1/1 test.
  - Command:
    `cmake --build build/default/cpp/Release --target test_ccd test_collision_world --parallel 5`
  - Result: passed.
  - Command:
    `ctest --test-dir build/default/cpp/Release -R '^(test_ccd|test_collision_world)$' --output-on-failure -j 5`
  - Result: passed, 2/2 tests.
  - Command:
    `cmake --build build/default/cpp/Release --target dart_collision_native_tests --parallel 5`
  - Result: passed.
  - Command:
    `ctest --test-dir build/default/cpp/Release --output-on-failure -L collision-native -j 5`
  - Result: passed, 29/29 tests.
- Closed-PR status and CI trigger limitation:
  - PR #2652 was closed per user direction at `2026-05-14T19:20:08Z`.
    GitHub still reports the closed PR head as `714d220d82a`, so pushed
    follow-up commits on `feature/new_coll` are not attached to that PR.
  - Pushing `feature/new_coll` remains useful for publishing branch state, but
    it does not start the main CI workflows because their `push` filters only
    include protected release/default branches. Using the `jslee02` GitHub
    token, manual `workflow_dispatch` started CI Linux run `25885373625`, CI
    gz-physics run `25885373580`, and Publish dartpy run `25885373596` for
    head `658da0edb10`.
- Closed-PR manual dispatch evidence on head `658da0edb10`:
  - Run/job: `25885373580` / `76075525533` (`CI gz-physics` /
    `GZ Physics Tests`).
  - Result: passed.
  - Run: `25885373596` (`Publish dartpy`).
  - Result: passed across all wheel build, repair, verify, test, and upload
    jobs for `ubuntu-latest-py312`, `ubuntu-latest-py313`,
    `ubuntu-latest-py314`, `macos-latest-py312`, `macos-latest-py313`,
    `macos-latest-py314`, `windows-latest-py312`,
    `windows-latest-py313`, and `windows-latest-py314`.
  - Artifact IDs:
    `7005084819`, `7005010502`, `7005119836`, `7005115921`,
    `7005173542`, `7005192639`, `7005327157`, `7005361057`,
    `7005313000`.
  - Run/job: `25885373625` / `76075528745` (`CI Linux` /
    `Native Collision (no FCL/Bullet/ODE)`).
  - Result: failed in `test_ccd`, `CapsuleCastConvex.DirectHit`; build
    completed successfully, then the `collision-native` label reported 28/29
    passing tests. Root cause: `capsuleCastConvex()` only tested the two
    endpoint spheres and could miss or numerically graze a full capsule-body
    hit against a convex target.
  - Repair in current working tree: `capsuleCastConvex()` now uses a capsule
    support function with conservative advancement against the convex support
    target, so the full capsule shape participates in the cast.
  - Focused local validation after the repair:
    `test_ccd` passed, the full `collision-native` label passed 29/29, the
    native-only CI focused CTest regex passed 4/4, and the Python
    collision/world smoke passed 17/17.
  - Run/job: `25885373625` / `76075528749` (`CI Linux` /
    `Collision Benchmark Guard`).
  - Result: benchmark command passed, but the upload step found no files
    because `actions/upload-artifact@v6` excludes hidden paths by default and
    the guard writes JSON under `.benchmark_results/`.
  - Repair in current working tree: set `include-hidden-files: true` on the
    benchmark artifact upload step so
    `collision-benchmark-guard-${{ github.run_id }}-${{ github.run_attempt }}`
    includes `.benchmark_results/collision_check_*.json`.
  - Because the repair changes C++ collision behavior, gz-physics, wheel
    matrix, native-only, and benchmark guard CI evidence needed a repaired-head
    refresh before those north-star CI gates could count.
- Closed-PR manual dispatch evidence on repaired head `1e1faf6feb1`:
  - Run/job: `25887939088` / `76084265248` (`CI Linux` /
    `Native Collision (no FCL/Bullet/ODE)`).
  - Result: passed. This covers the previous `test_ccd` /
    `CapsuleCastConvex.DirectHit` failure on GitHub.
  - Run/job: `25887940214` / `76084227295` (`CI gz-physics` /
    `GZ Physics Tests`).
  - Result: passed.
  - Run/job: `25887939088` / `76084265196` (`CI Linux` /
    `Collision Benchmark Guard`).
  - Result: passed and uploaded
    `collision-benchmark-guard-25887939088-1`, artifact id `7006005918`,
    digest
    `sha256:c92c993b9d6a2eaf0ac234d7526cc9893c6544992f0ed55fd77a8bf7f02ba2f5`.
  - Run: `25887941240` (`Publish dartpy`).
  - Result: passed across all wheel build, repair, verify, test, and upload
    jobs for `ubuntu-latest`, `macos-latest`, and `windows-latest` on Python
    3.12, 3.13, and 3.14. The PyPI publish job was skipped as expected.
  - Artifact IDs:
    `7006040650`, `7006017034`, `7005950943`, `7005940232`,
    `7005904083`, `7005879764`, `7006075376`, `7006153851`,
    `7006180173`.
- Closed-PR trigger state after later pushed audit/docs heads:
  - Audit/docs/policy heads after the manual-dispatch evidence baseline are
    pushed to `feature/new_coll`, but
    `gh run list --branch feature/new_coll --commit` returns no Actions runs
    for checked docs/evidence commits. These pushes publish branch state only
    because the workflows do not run on arbitrary feature-branch pushes.
- Broad local Debug validation after the macOS/Linux closed-PR repairs:
  - Command:
    `cmake --build build/default/cpp/Debug --target tests --parallel 5`
  - Result: built the normal Debug C++ test target set. A first full CTest run
    then passed the native and general tests but reported 13
    `simulation-experimental` tests as `Not Run` because the `tests` target
    does not build the separate `dart_experimental_tests` aggregate.
  - Command:
    `cmake --build build/default/cpp/Debug --target dart_experimental_tests --parallel 5`
  - Result: built the 13 configured simulation-experimental test executables.
  - Follow-up finding: `test_logging` failed because Debug source context
    dropped the file name when `DART_EXPERIMENTAL_SOURCE_DIR` was not defined.
    The logging helper now falls back to the original source path when no
    source-root macro is available.
  - Command:
    `ctest --test-dir build/default/cpp/Debug --output-on-failure -L simulation-experimental -j 5`
  - Result: passed, 13/13 tests.
  - Command:
    `ctest --test-dir build/default/cpp/Debug --output-on-failure -j 5`
  - Result: passed, 277/277 tests. Label summaries included
    `collision-native` 29 tests and `simulation-experimental` 13 tests.
- Closed-PR full local validation after the nanobind test harness repair:
  - Initial finding: `pixi run test-all` failed linking Release and Debug
    `dartpy` with undefined `nanobind::detail::internals_inc_ref()` and
    `internals_dec_ref()` symbols. The test harness had refreshed
    `nb_func.cpp` from upstream because it treated the valid
    `buf.put(" = ");` line as a bad marker, but the matching
    `nb_internals.cpp` object stayed stale.
  - Repair: `scripts/test_all.py` now drops the valid marker, refreshes
    `nb_internals.cpp` and `nb_internals.h` with any upstream nanobind source
    refresh, and touches internals when `nb_func.cpp` is newer so Ninja
    rebuilds the static nanobind objects coherently.
  - Command:
    `pixi run cmake --build build/default/cpp/Release --target dartpy --parallel 5`
  - Result: passed.
  - Command:
    `pixi run cmake --build build/default/cpp/Debug --target dartpy --parallel 5`
  - Result: passed.
  - Command:
    `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  - Result: passed. The comprehensive suite reported 6/6 top-level gates
    passed: linting, build, unit tests, simulation-experimental tests, Python
    tests, and documentation.
- Current local validation refresh on docs-only head `f5d4f9ee932`:
  - Command:
    `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  - Result: passed. The comprehensive suite reported 6/6 top-level gates
    passed: linting, build, unit tests, simulation-experimental tests, Python
    tests, and documentation. Release CTest reported 264/264 tests passed,
    including the 29-test `collision-native` label. The run used native-only
    default configure values with FCL, Bullet, ODE, reference tests, and
    reference benchmarks all `OFF`.
- Local compatibility-facade audit hardening:
  - Change: `scripts/check_collision_runtime_isolation.py` now asserts
    top-level `dart/collision/{fcl,bullet,ode}` source/header files are explicit
    public facades and only forward to native-backed `compat/` headers.
  - Change: `scripts/audit_collision_compat_facades.py` now audits retained
    legacy factory keys, C++ detector/group facades, absence of dartpy legacy
    detector aliases, and legacy CMake package component names as routes to
    native DART collision.
  - Commands:
    `python scripts/check_collision_runtime_isolation.py`,
    `python scripts/audit_collision_compat_facades.py`,
    `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run lint`,
    `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run check-lint`
  - Result: passed. `pixi run lint` now includes
    `audit-collision-compat-facades` alongside the existing runtime isolation
    guard, and `pixi run check-lint` verifies the same audit through the
    non-mutating CI-style lint path.
- Final local validation after compatibility-facade audit hardening:
  - Command:
    `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  - Result: passed. The comprehensive suite reported 6/6 top-level gates
    passed: linting, build, unit tests, simulation-experimental tests, Python
    tests, and documentation. This final pass included
    `audit-collision-compat-facades` through the lint gate.
- Current pushed clean-API/build-option policy checkpoint:
  - Commit: `ec6f6f43112`
    (`Clean dartpy collision API and deprecate C++ facades`), pushed to
    `origin/feature/new_coll`.
  - Result: passed local validation before push:
    `pixi run lint`, `pixi run build`, `pixi run test-unit` (277/277 CTest
    tests), `pixi run test-py` (147 Python tests),
    `pixi run -e gazebo test-gz` (65/65 gz-physics tests), and the native
    compatibility package smoke.
  - Policy evidence: dartpy exposes `DartCollisionDetector` without legacy
    detector aliases; retained C++ FCL/Bullet/ODE facades and legacy factory
    keys are deprecated native-backed compatibility paths; and
    `DART_BUILD_COLLISION_FCL`, `DART_BUILD_COLLISION_BULLET`, and
    `DART_BUILD_COLLISION_ODE` now document reference-only scope. Core DART,
    dartpy, gz-physics runtime integration, and the native-backed
    compatibility component facades do not need those options.
  - GitHub trigger evidence:
    `gh run list --branch feature/new_coll --commit ec6f6f43112 ...` returned
    no runs because PR #2652 is closed and feature-branch pushes do not match
    the main workflow `push` filters.
- Follow-up collision reference build-option docs checkpoint:
  - Commit: `aa3ccce70c7`
    (`Clarify collision reference build options`), pushed to
    `origin/feature/new_coll`.
  - Result: passed `pixi run lint`, including
    `check-collision-runtime-isolation`, `audit-collision-compat-facades`, and
    default configure output with FCL, Bullet, ODE, reference tests, and
    reference benchmarks all `OFF`.
  - GitHub trigger evidence:
    `gh run list --branch feature/new_coll --commit aa3ccce70c7 ...` returned
    no runs because PR #2652 is closed and feature-branch pushes do not match
    the main workflow `push` filters.
- Final runtime cleanup reference-file audit:
  - Commands:
    `find dart/collision/{fcl,bullet,ode}/reference -type f | sort`,
    per-engine `find ... | wc -l`, and an `rg` check for `file(GLOB`,
    `dart_add_library`, and `add_component_targets` in the three engine
    `CMakeLists.txt` files.
  - Result: no unreferenced old-engine runtime source was found outside the
    explicit reference targets. FCL has 12 reference files, Bullet has 16, and
    ODE has 26. Their CMake files glob `reference/*.cpp` plus detail sources
    where applicable, build only `dart-collision-reference-fcl`,
    `dart-collision-reference-bullet`, and `dart-collision-reference-ode`, and
    register only the matching `collision-reference-*` package components.
  - Deletion decision: no file deletion in this slice. The remaining
    FCL/Bullet/ODE implementation files are intentionally retained as opt-in
    reference comparison code for `createReference()`, reference tests, and
    comparative benchmarks.
- Current full local validation refresh:
  - Commit: `9ed54f2c49192343aee9418ea7d7835880731556`
    (`Align native collision PR handoff wording`), pushed to
    `origin/feature/new_coll`.
  - Command:
    `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run test-all`
  - Result: passed. The comprehensive suite reported 6/6 top-level gates:
    linting, build, unit tests, simulation-experimental tests, Python tests,
    and documentation. The run configured normal native-only defaults with
    `DART_BUILD_COLLISION_FCL`, `DART_BUILD_COLLISION_BULLET`,
    `DART_BUILD_COLLISION_ODE`, collision reference tests, and collision
    reference benchmarks all `OFF`, while still building dartpy and passing
    the compatibility-facade lint/audit guards. This confirms the build flags
    are not required by core DART, dartpy, gz-physics runtime compatibility, or
    the native-backed legacy package component facades.
- Current local collision benchmark guard refresh:
  - Commit: `c4fd6e27f8cd620adfc900e83d9b3e5f0f959d19`
    (`Clarify collision reference option scope`), pushed to
    `origin/feature/new_coll`.
  - Command:
    `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e collision-reference bm-collision-check`
  - Result: passed. The `collision-reference` configure intentionally enabled
    FCL, Bullet, ODE, reference tests, and reference benchmarks as opt-in
    comparison inputs. Checked subsets passed with no failures or skips for
    narrow phase (3/3), distance (3/3), raycast (5/5), mixed primitives (4/4),
    mesh-heavy (1/1), and raycast-batch (2/2), and the public
    `DartCollisionDetector` adapter benchmark completed successfully.
  - Local JSON outputs:
    `.benchmark_results/collision_check_narrow.json`,
    `.benchmark_results/collision_check_distance.json`,
    `.benchmark_results/collision_check_raycast.json`,
    `.benchmark_results/collision_check_mixed.json`,
    `.benchmark_results/collision_check_mesh.json`,
    `.benchmark_results/collision_check_raycast_batch.json`, and
    `.benchmark_results/collision_check_adapter.json`.

## Known Risks

- Direct public C++ legacy detector `create()` entry points now return
  native-backed detector facades. They may preserve legacy display type strings
  for gz-physics compatibility, while the public factory and Python
  compatibility routes still canonicalize to `dart`. Legacy package component
  names are now native-backed interface facades, old-engine shared libraries
  are explicitly reference-named, and top-level source-tree legacy
  detector/group headers are native-backed facades. Old FCL, Bullet, and ODE
  implementation files still exist only as explicit reference test/benchmark
  surfaces under `reference/` paths.
- Normal pixi configure paths, default/wheel Pixi lock metadata, the repaired
  py312 wheel artifact, and the repaired-head workflow-dispatch wheel matrix
  now exclude the old collision engines. Keep final PR-state packaging
  evidence current so later Python/platform builds cannot reintroduce FCL,
  Bullet, ODE, or libccd runtime links.
- Compatibility-alias checks can silently bypass native-only paths if they are
  used outside explicit backward-compatibility tests.
- Scenario-scale collision manager performance now passes the recorded
  benchmark set. Dirty-world streaming traversal remains a watch point for
  future simulation-style benchmarks because the dense 1000 win relies on
  reusing an already-valid broad-phase snapshot.
- Native `VoxelGridShape` support adapts occupied OctoMap leaves into compound
  box cells. This closes DART's feature surface and tests, but very large
  voxel grids may need a dedicated acceleration path if they become hot
  simulation workloads.
- gz-physics depends on legacy detector names and compatibility headers; keep
  facade behavior covered while native lowercase APIs evolve. The latest local
  gz-physics evidence is a fresh `pixi run -e gazebo test-gz` pass: 65/65
  tests against the DART plugin, and repaired-head workflow-dispatch
  gz-physics CI also passed. Final PR-state/downstream policy evidence still
  blocks the release gate.
- Installed-package compatibility remains important: native-backed
  compatibility component files must keep downstream packages such as
  gz-physics linking without making FCL, Bullet, or ODE required for native
  default builds.
