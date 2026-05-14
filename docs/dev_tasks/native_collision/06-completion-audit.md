# Native Collision Completion Audit

This audit maps the user-facing north-star requirements to concrete artifacts
and evidence. It is the checkpoint to read before deciding whether
`docs/dev_tasks/native_collision/` can be deleted.

## Objective Restatement

The single PR is complete only when DART's normal collision runtime is one
built-in detector stack, FCL/Bullet/ODE are absent from normal runtime builds,
old detector names are compatibility facades over the built-in detector, and
the built-in layer is proven feature-complete, correct, scalable, and
performance-oriented through tests, benchmarks, package checks, downstream
compatibility, and CI artifacts.

## Completion Decision

Status: not complete.

The current branch has strong local evidence, but the north star still has
unverified external and finalization gates:

- Native-only and gz-physics CI evidence is still missing.
- Full dartpy wheel matrix artifact evidence is still missing.
- GitHub artifact evidence for the scheduled/manual collision benchmark guard
  is still missing.
- Draft PR #2652 now exists and CI has started, but the first CI refresh is not
  green yet. The initial `Asserts enabled (no -DNDEBUG)` Linux job exposed a
  workflow mismatch: it manually enabled Bullet in the default Pixi
  environment, where Bullet is intentionally absent after the native-only
  dependency cleanup. The workflow repair was pushed as `901d56c4260` and
  configures that custom asserts build as native-only unless it installs a
  reference-engine environment. A follow-up `origin/main` merge was committed
  locally as `3120a4fce9b` and pushed to clear PR mergeability conflicts, with
  focused local configure, build, and anchored CTest evidence already passing
  for the merge-resolved collision, contact, world, and GUI targets.
  The next PR refresh found a Windows-only Pixi parser failure in
  `CI Windows` / `Tests (Release)` before CMake ran; Windows `config`,
  `config-py`, and `config-install` now use `bash -lc` so the native-only
  collision override defaults are interpreted by Bash rather than by Pixi's
  Windows shell parser. The following PR refresh reached past that parser
  failure and exposed an Alt Linux configure failure in
  `CI Alt Linux (Docker)` / `Alt Linux repro (Docker)`: Eigen 5 satisfies
  DART's minimum Eigen requirement, but `find_package(Eigen3 3.4 ...)` rejects
  a different major version. The experimental dependency lookup now follows the
  core Eigen finder pattern by finding Eigen3 first and then enforcing the
  explicit `Eigen3_VERSION >= 3.4` check. The next PR refresh exposed a
  Windows wheel compile failure in `Wheels | windows-latest Py312`:
  `dart-collision-native` compiled without `/utf-8`, which conda-forge `fmt`
  now requires for MSVC Unicode support. The root MSVC C++ flags now include
  `/utf-8` so normal builds and wheel builds share the fix. The same PR
  refresh showed Alt Linux got past Eigen and failed on missing EnTT package
  config; `dart-collision-native` now finds only its direct Eigen/EnTT
  dependencies and uses the EnTT FetchContent fallback when a system package is
  unavailable.
  The first EnTT fallback repair got past package discovery but failed CMake
  generation because the fetched EnTT build target was not in DART's export
  set. The fallback now populates EnTT headers only and exposes `EnTT::EnTT` as
  an imported interface target. The next PR refresh on head `97a9d3ca6d6`
  reached macOS release arm64 compilation and failed because new native
  collision test/benchmark files still included generated CamelCase
  compatibility headers; those includes now use lowercase canonical headers
  and explicit lowercase `reference/` detector headers. The next refresh on
  head `623f0180fa8` reached macOS Debug linking and failed
  `UNIT_collision_DistanceFilter` because `BodyNodeDistanceFilter` still had
  public declarations but no compiled implementation; the current repair
  restores the body-node distance filter definitions. The following refresh on
  head `f726ee5e4ee` reached macOS Release compilation and failed
  `UNIT_collision_NativeBackend` because `DistanceFilter` was a polymorphic
  shared-pointer base without a virtual destructor; that base interface now has
  a defaulted virtual destructor. The same refresh showed
  `Wheels | macos-latest Py314` successfully building and delocating the wheel,
  then rejecting the native-backed `dart_collision-{fcl,bullet,ode}Component`
  compatibility facade files. The wheel collision-isolation verifier now keeps
  old runtime/reference artifacts forbidden while allowing those facade
  component files. The next refresh on head `714d220d82a` reached further into
  macOS arm64 test coverage: Release failed on unused variables in native
  collision filter callback tests, while Debug exposed a native sphere-box CCD
  starting-inside assertion, a VSG headless-backend availability assumption, and
  a world collision-detector fallback test that overrode the `"dart"` creator
  before constructing the world. The current repairs remove the unused
  variables, handle initial-overlap CCD without indexing an unset axis, skip
  VSG simple-viewer tests when headless creation throws on a runner, and make
  the world fallback test construct its current detector before overriding the
  factory. The same closed-PR Linux run also exposed a Debug-only
  `test_collision_world` dangling-pointer issue in world-level sphere/capsule
  cast results; the world query layer now keeps stable collision-object handles
  for raycast, sphere-cast, and capsule-cast result pointers.
- Downstream migration/deprecation evidence is still missing.
- Final compatibility-facade retention/deprecation evidence is still missing:
  the documented decision is to delete old external-engine runtime
  implementations, keep only native-backed compatibility facades required by
  downstream migration, and leave FCL/Bullet/ODE access in explicit
  reference-only test/benchmark APIs.
- Full local `pixi run test-all` evidence is refreshed for the current local
  state after fixing one stale VSG native `ShapeType` switch left by the native
  taxonomy cleanup. Final `pixi run test-all` evidence after the eventual
  PR-complete state is still missing. The refreshed full reruns found and
  repaired two local validation robustness gaps: a stale optional `libccd`
  CMake cache issue in the default native-only test build, and stale
  `ShapeType::Cone`, `ShapeType::HeightField`, and `ShapeType::PointCloud`
  cases in the VSG geometry builder after those native tags were removed.
- The dev-task folder must remain until final PR evidence is transferred to
  the PR description and the folder is deleted in the completing PR. Durable
  architecture notes are now seeded in onboarding docs, but final evidence
  transfer is still pending.

## Audit Inputs

Current audited state:

- Branch: `feature/new_coll`
- Latest published checkpoint before CI repair: `5b08a00d381`
  (`Record current native collision validation pass`), which records the
  current local validation pass after the durable built-in architecture docs
  and compatibility-facade policy cleanup.
- Remote branch state: `origin/feature/new_coll` is published at
  the latest pushed `feature/new_coll` head after the asserts-enabled workflow
  repair and follow-up `origin/main` merge. The merge commit `3120a4fce9b`
  clears the previous GitHub mergeability conflicts.
- GitHub PR state: draft PR #2652
  (https://github.com/dartsim/dart/pull/2652) targets `main`, has the
  `DART 7.0` milestone, and is collecting CI evidence. The PR is intentionally
  draft while the audit gates remain open.
- Initial PR CI state: GitHub Actions started for the pushed head. The first
  completed failure was `Asserts enabled (no -DNDEBUG)`, run `25870574281`,
  job `76024440344`, which failed during configure because
  `DART_BUILD_COLLISION_BULLET=ON` was still set in the default Pixi
  environment. The pushed repair changes that custom configure to keep
  FCL/Bullet/ODE and collision reference tests/benchmarks `OFF`.
- Follow-up PR CI state after the Windows parser repair: run `25873507050`,
  job `76034633035` (`CI Alt Linux (Docker)` /
  `Alt Linux repro (Docker)`) failed during CMake configure because Alt Linux
  provides Eigen 5.0.0 and the experimental dependency helper requested
  `find_package(Eigen3 3.4 ...)` directly. The current repair removes the
  requested version from that `find_package` path and keeps DART's explicit
  `>=3.4` post-find check.
- Follow-up PR CI state after the Alt Linux Eigen repair: run `25874275078`,
  job `76037407583` (`Wheels | windows-latest Py312`) failed while compiling
  the static `dart-collision-native` wheel target because MSVC lacked `/utf-8`
  and `fmt/base.h` emitted `static_assert failed: 'Unicode support requires
compiling with /utf-8'`. The current repair adds `/utf-8` to the root MSVC
  C++ flags.
- Follow-up PR CI state for Alt Linux after the Eigen repair: run
  `25874678072`, job `76038637120` (`CI Alt Linux (Docker)` /
  `Alt Linux repro (Docker)`) got past Eigen and failed during configure on
  missing `EnTTConfig.cmake`. The current repair decouples
  `dart-collision-native` from the full simulation-experimental dependency
  bundle and adds an EnTT FetchContent fallback through
  `cmake/dart_find_entt.cmake`.
- Follow-up PR CI state after the first EnTT fallback repair: run
  `25875314192`, job `76040846463` got past missing EnTT and failed during
  CMake generation because `install(EXPORT "dart_component_collision-native"
...)` saw the fetched EnTT build target outside DART's export set. The
  current repair makes the fallback header-only from DART's perspective by
  populating EnTT sources and creating an imported `EnTT::EnTT` interface
  target.
- Follow-up PR CI state after the export-safe EnTT fallback repair: run
  `25875811218`, job `76044208533` (`CI macOS` /
  `Release Tests (arm64)`) failed compiling
  `INTEGRATION_collision_native_backend_consistency` because several new native
  collision integration/benchmark sources included generated CamelCase
  compatibility headers. The current repair replaces those includes with
  canonical lowercase headers and explicit lowercase reference-backend headers.
- Follow-up PR CI state after the macOS include repair: run `25877526350`,
  job `76048820253` (`CI macOS` / `Debug Tests (arm64)`) failed linking
  `UNIT_collision_DistanceFilter` because `BodyNodeDistanceFilter` declared
  `needDistance()` and `areAdjacentBodies()` but
  `dart/collision/distance_filter.cpp` did not define them. The current repair
  restores the historical body-node distance filtering semantics, with focused
  Debug and Release local build/CTest coverage for
  `UNIT_collision_DistanceFilter`.
- Follow-up PR CI state after the body-node distance filter repair: run
  `25878909676`, job `76053447798` (`CI macOS` /
  `Release Tests (arm64)`) failed compiling `UNIT_collision_NativeBackend`
  because `DistanceFilter` had virtual methods but no virtual destructor while
  tests delete a subclass through `std::shared_ptr<DistanceFilter>`. The
  current repair adds the base virtual destructor, with focused Release
  `UNIT_collision_NativeBackend` build and CTest coverage passing locally.
- Follow-up PR CI state for the py314 macOS wheel: run `25878909684`, job
  `76053782968` (`Wheels | macos-latest Py314`) failed in `wheel-verify`
  after delocation because `verify_wheel_collision_isolation.py` still
  rejected native-backed FCL/Bullet/ODE compatibility component facade files.
  The current repair allows those facade files while still rejecting old
  runtime libraries, reference libraries, and reference component exports. A
  focused synthetic wheel probe and the existing repaired py312 Linux wheel
  pass locally with the updated policy, while synthetic reference/runtime
  artifacts are still rejected.
- Follow-up PR CI state on head `714d220d82a`: run `25880337655`, job
  `76058385300` (`CI macOS` / `Release Tests (arm64)`) failed compiling
  `test_collision_filter.cpp` because callback-filter tests retained unused
  local object variables. The current repair removes the unused bindings.
- Follow-up PR CI state on the same head: run `25880337655`, job
  `76058385388` (`CI macOS` / `Debug Tests (arm64)`) failed
  `test_ccd`, `UNIT_gui_vsg_simple_viewer`, and
  `INTEGRATION_simulation_World`. Focused local Debug reproduction showed
  native sphere-box CCD asserted when `hitAxis` stayed unset for an
  initial-overlap cast, VSG simple-viewer tests assumed headless construction
  always succeeds, and the world typed-setter test made the default world
  constructor use a null `"dart"` factory creator before testing setter
  fallback. The current repair handles initial-overlap CCD, skips VSG
  simple-viewer tests when headless creation throws, and moves the world
  factory override after world construction.
- Follow-up PR CI state on the same Linux head: run `25880337619`, job
  `76058493678` (`CI Linux` / `Debug Tests`) failed
  `test_collision_world` after returning sphere-cast and capsule-cast results
  whose `object` pointers referenced loop-local `CollisionObject` handles.
  The current repair caches stable world-owned query handles for world-level
  raycast, sphere-cast, and capsule-cast results.
- Follow-up PR CI state on the same Linux head: run `25880337619`, job
  `76058493716` (`CI Linux` / `Native Collision (no FCL/Bullet/ODE)`) failed
  inside the `collision-native` label. The current local Release
  `collision-native` label passes after the CCD and query-result repairs.
- Follow-up local validation after the audit started from
  `1da52368282` and found that `pixi run test-all` failed in `build-tests`
  because a stale cached `LIBCCD_LIBRARY` pointed at a removed
  `.pixi/envs/default/lib/libccd.so`. The working-tree repair resets missing
  cached optional libccd include/library paths, and
  `pixi run build-tests ON Release` passes with the optional
  `test_libccd_algorithms` target excluded when libccd is unavailable.
- Follow-up full local validation at `da532729bec`:

  ```bash
  DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all
  ```

  Result: passed before the later native shape-taxonomy cleanup. The full suite
  reported lint, build, unit tests, simulation-experimental tests, Python
  tests, and documentation all passed, with Release CTest passing 264/264 tests
  including 29 `collision-native` label tests.

- Follow-up focused validation at `cd16b64be0b`:

  ```bash
  cmake --build build/default/cpp/Release --target test_shapes --parallel $(python scripts/parallel_jobs.py)
  ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_shapes$'
  cmake --build build/default/cpp/Release --target dart_collision_native_tests --parallel $(python scripts/parallel_jobs.py)
  ctest --test-dir build/default/cpp/Release --output-on-failure -L collision-native
  pixi run lint
  ```

  Result: passed. The shape taxonomy cleanup removed unused native
  `ShapeType` tags for cone, heightfield, and point-cloud while keeping those
  DART shapes represented through adapter-owned convex, mesh, compound, or
  non-collidable behavior.

- Follow-up full local validation after the VSG shape-type switch repair:

  ```bash
  pixi run -- cmake --build build/default/cpp/Debug --target dart-gui-vsg --parallel $(python scripts/parallel_jobs.py)
  DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all
  ```

  Result: passed. The focused Debug VSG build rebuilt
  `geometry_builders.cpp` and linked `libdart-gui-vsgd`. The full test-all
  rerun reported lint, build, unit tests, simulation-experimental tests,
  Python tests, and documentation all passed.

- Follow-up full local validation at `864dd56d944c` after the durable
  architecture docs and compatibility-facade policy cleanup:

  ```bash
  DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all
  ```

  Result: passed. The comprehensive suite reported 6/6 top-level gates passed:
  linting, build, unit tests, simulation-experimental tests, Python tests, and
  documentation. The final report printed `All tests passed!` and
  `Ready to submit PR!`.

- Closed-PR follow-up status after user direction to avoid creating a new
  diff:
  - PR #2652 is closed and remains anchored to old head `714d220d82a`.
  - Follow-up fixes are being pushed to `feature/new_coll`, but branch pushes
    do not start the main workflows because the workflow `push` filters only
    include protected release/default branches. Manual workflow dispatch was
    attempted for the relevant workflows and failed with HTTP 403 requiring
    repository admin rights.

- Follow-up broad local Debug validation after the closed-PR macOS/Linux
  repairs:

  ```bash
  cmake --build build/default/cpp/Debug --target tests --parallel 5
  cmake --build build/default/cpp/Debug --target dart_experimental_tests --parallel 5
  ctest --test-dir build/default/cpp/Debug --output-on-failure -L simulation-experimental -j 5
  ctest --test-dir build/default/cpp/Debug --output-on-failure -j 5
  ```

  Result: passed after one additional simulation-experimental logging repair.
  The first full CTest attempt showed the native/general Debug tests green but
  exposed that `tests` does not build the separate
  `dart_experimental_tests` aggregate; after building it, `test_logging`
  failed because Debug source context discarded the file name when
  `DART_EXPERIMENTAL_SOURCE_DIR` was not defined. The logging fallback now
  preserves the original source path without that macro. The final full Debug
  CTest rerun passed 277/277 tests, including 29 `collision-native` tests and
  13 `simulation-experimental` tests.

- Local command run during this audit:

  ```bash
  python scripts/check_collision_runtime_isolation.py
  ```

- Observed output:

  ```text
  Collision runtime isolation check passed.
  ```

Additional inspected artifacts:

- `docs/dev_tasks/native_collision/README.md`
- `docs/dev_tasks/native_collision/01-design.md`
- `docs/dev_tasks/native_collision/03-evidence-gates.md`
- `docs/dev_tasks/native_collision/04-reference-gap-analysis.md`
- `docs/dev_tasks/native_collision/05-downstream-migration.md`
- `docs/onboarding/README.md`
- `docs/onboarding/architecture.md`
- `docs/onboarding/build-system.md`
- `pixi.toml`
- `.github/workflows/ci_ubuntu.yml`
- `.github/workflows/publish_dartpy.yml`
- `scripts/check_collision_runtime_isolation.py`
- `scripts/verify_wheel_collision_isolation.py`
- `dart/collision/**`
- `python/dartpy/collision/collision_detector.cpp`
- `tests/unit/collision/**`
- `tests/benchmark/collision/**`
- `docs/dev_tasks/native_collision/smoke/native_compat_package/**`

## Prompt-To-Artifact Checklist

| Requirement                                                                                                                  | Evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Status  |
| ---------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- |
| Track progress on the scale of the path toward the north star.                                                               | `README.md` and `03-evidence-gates.md` both define the 0-12 north-star progress scale. Stage 10 has local design evidence, stage 11 has local source/reference split evidence, and stage 12 remains blocked on CI, migration, final deletion, validation, and PR packaging.                                                                                                                                                                                              | Done    |
| One PR continues until it fully reaches the north star.                                                                      | `README.md` states the checkpoint is not a final PR boundary, and `02-milestones.md` keeps final PR packaging and dev-task deletion as completion gates.                                                                                                                                                                                                                                                                                                                 | Open    |
| `dart/collision/` stops exposing a real multi-backend runtime selection layer.                                               | `dart/collision/dart/dart_collision_detector.hpp` owns the canonical `dart` registrar plus legacy alias registrars. Top-level FCL/Bullet/ODE headers route through compatibility facades, and old implementation sources live under explicit `reference/` paths.                                                                                                                                                                                                         | Local   |
| Retained FCL/Bullet/ODE/experimental names are wrappers/adapters over the built-in detector.                                 | Compatibility headers under `dart/collision/{fcl,bullet,ode}/compat/`, Python aliases in `python/dartpy/collision/collision_detector.cpp`, and the package smoke under `docs/dev_tasks/native_collision/smoke/native_compat_package/` verify native-backed compatibility names.                                                                                                                                                                                          | Local   |
| Any selected "backend" through retained public names always uses the built-in detector.                                      | `UNIT_collision_DartCollisionDetector` covers factory aliases and legacy facade behavior. The native compatibility package smoke verifies package components, factory keys, installed headers, and direct legacy `create()` calls.                                                                                                                                                                                                                                       | Local   |
| FCL/Bullet/ODE remain only as optional reference engines for tests and benchmarks.                                           | `DART_BUILD_COLLISION_REFERENCE_TESTS` and `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` exist in CMake/Pixi configure paths; `collision-reference` is the explicit opt-in Pixi environment; reference call sites use `createReference()` and `collision-reference-*` names.                                                                                                                                                                                               | Local   |
| Native-only builds can opt out of FCL/Bullet/ODE.                                                                            | Normal Pixi configure paths default FCL, Bullet, ODE, reference tests, and reference benchmarks to `OFF`; local native-only build/install/wheel evidence is recorded in `03-evidence-gates.md`.                                                                                                                                                                                                                                                                          | Local   |
| Default packages and wheels do not carry old collision runtime dependencies.                                                 | Package install probes, Pixi dependency metadata checks, and the `verify_wheel_collision_isolation.py` verifier are recorded. `wheel-verify-core` runs the wheel isolation verifier, and `publish_dartpy.yml` invokes `pixi run -e py${{ matrix.python-version }}-wheel wheel-verify`.                                                                                                                                                                                   | Local   |
| Built-in detector maintains correctness tests.                                                                               | Native unit/integration tests, feature parity tests, DART adapter tests, gz-focused regressions, and reference comparison tests are recorded in `03-evidence-gates.md`.                                                                                                                                                                                                                                                                                                  | Local   |
| Built-in detector maintains benchmarks for performance optimization.                                                         | `pixi run -e collision-reference bm-collision-check` runs checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy, raycast-batch, and public adapter benchmark subsets, writing `.benchmark_results/collision_check_*.json`.                                                                                                                                                                                                                                 | Local   |
| Native performance should beat the best legacy engine on required workloads, with feature first and correctness preserved.   | The current local broad benchmark guard passed at `892e50d02e4`; `README.md` records native wins on the measured primitive, narrowphase, supported distance, raycast, batch, mesh-heavy, and mixed workloads.                                                                                                                                                                                                                                                            | Local   |
| Built-in layer architecture must be API-clean, scalable, and performance-oriented.                                           | `01-design.md` defines the public API boundary, compatibility shell, DART adapter scene, native scene/query core, query lifecycle, scalability design, performance hooks, and reference harness boundary. `README.md` mirrors this in the Architecture Completion Rubric, `docs/onboarding/architecture.md` now carries the durable built-in runtime architecture summary, and native `ShapeType` cleanup keeps the native taxonomy aligned with real supported classes. | Local   |
| Architecture evidence must cover code, tests, package/source boundaries, and benchmark/profiling hooks, not only prose docs. | Code evidence exists for factory aliases, compatibility facades, persistent `DartCollisionGroup` scene state, cache IDs, source isolation, native shape taxonomy cleanup, wheel verifier wiring, package smoke, adapter benchmarks, and native/reference benchmark JSON. CI/artifact evidence remains missing.                                                                                                                                                           | Partial |
| gz-physics compatibility must be preserved while legacy names migrate.                                                       | Fresh local `pixi run -e gazebo test-gz` passed 65/65 and package smoke passed. `05-downstream-migration.md` defines the compatibility contract and removal gates.                                                                                                                                                                                                                                                                                                       | Local   |
| Downstream migration/deprecation path must be proven before removing retained facades.                                       | `05-downstream-migration.md` defines the migration order and gates. DART-side package smoke is local evidence only; downstream CI/deprecation evidence is still missing.                                                                                                                                                                                                                                                                                                 | Open    |
| Final PR evidence and cleanup must happen in the same PR.                                                                    | `README.md`, `02-milestones.md`, and `docs/dev_tasks/README.md` require transferring final evidence to the PR description and deleting `docs/dev_tasks/native_collision/` only at completion. Durable collision architecture notes have been seeded in onboarding docs, but PR evidence transfer and folder deletion remain open.                                                                                                                                        | Open    |

Legend:

- Done: documentation/process requirement is satisfied.
- Local: implemented and locally verified, but still needs final CI or PR
  artifact evidence before north-star completion.
- Partial: some real evidence exists, but coverage is not enough to close the
  requirement.
- Open: required evidence or finalization work is missing.

## Missing Evidence And Required Next Actions

1. Do not create a new diff or review request until the user asks. PR #2652 is
   closed, so pushing focused fixes to `feature/new_coll` publishes branch
   state but currently does not trigger the main GitHub Actions workflows.
   Authoritative CI evidence still needs a permitted trigger surface.
2. Collect CI run links and artifact names for:
   - native-only collision/default build jobs,
   - gz-physics compatibility jobs,
   - dartpy wheel jobs that run `wheel-verify`,
   - `Collision Benchmark Guard` JSON artifacts.
3. Record downstream migration/deprecation evidence proving gz-physics and
   downstream users no longer depend on legacy names as runtime backend
   selectors.
4. Apply the documented compatibility-facade policy in the final PR state:
   preserve only wrappers required for source compatibility, keep them
   native-backed, and keep all external engines reference-only.
5. Run final validation after the final code state, including at least
   `pixi run lint` and `pixi run test-all`, plus any CI-specific gates whose
   failures are not covered locally.
6. Keep the durable collision architecture summary in onboarding docs, move
   final command evidence into the PR description, then delete this dev-task
   folder in the same PR.

## Completion Bar

Do not mark this dev task complete until every row in the checklist is either
`Done` with durable docs evidence or backed by CI/PR artifacts that close the
remaining `Local`, `Partial`, and `Open` statuses. Until then,
`docs/dev_tasks/native_collision/` remains active working documentation.
