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

- Native-only and gz-physics manual workflow-dispatch evidence is now collected
  for pushed head `1e1faf6feb1`, but the final PR-complete state still needs
  whatever CI surface the maintainer chooses because PR #2652 is closed.
- Full dartpy wheel matrix artifact evidence is now collected for pushed head
  `1e1faf6feb1`, but final PR packaging still needs the evidence transferred
  to the PR description or successor review surface.
- GitHub artifact evidence for the manual collision benchmark guard is now
  collected for pushed head `1e1faf6feb1`; scheduled/permanent gate evidence is
  still a finalization item. Recent local benchmark-guard evidence is
  refreshed on `1f3505ea469`, but final GitHub artifact evidence still needs a
  maintainer-selected PR or workflow surface.
- PR #2652 was used as the initial CI surface, then closed per user direction
  before the latest follow-up commits. The closed PR remains anchored to old
  head `714d220d82a`; feature-branch pushes do not match the main workflow
  `push` filters. Manual workflow-dispatch runs on `658da0edb10` passed
  gz-physics CI and the full dartpy wheel matrix, but Linux native-only CI
  exposed a `CapsuleCastConvex.DirectHit` failure. The pushed repair at
  `1e1faf6feb1` makes `capsuleCastConvex()` use a capsule support function
  with conservative advancement against convex targets and enables benchmark
  artifact upload from the hidden `.benchmark_results/` directory. Manual
  workflow-dispatch reference runs on that repaired head passed the
  native-only job, gz-physics, the full dartpy wheel matrix, and the benchmark
  guard artifact upload. The initial `Asserts enabled (no -DNDEBUG)`
  Linux job exposed a
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
- Later pushed audit/docs/policy heads contain audit evidence, CI trigger
  notes, clean dartpy collision API policy, reference build-option
  refinements, and public docs alignment for the built-in runtime collision
  path. The latest checked docs/evidence head before this audit-record update
  is `ad1b6782578`. The matching `gh run list` check returned no GitHub
  Actions runs because these branch pushes are not an allowed workflow trigger
  while PR #2652 is closed.
- Local downstream migration evidence is refreshed: fresh gz-physics,
  package-smoke, runtime-isolation, C++ compatibility-name, dartpy clean-API,
  direct link-inspection checks, and human-authored stale-backend docs audit all
  pass. The primary executable/link-smoke evidence is on `8c83cd19cb8`.
  Downstream deprecation policy evidence is still missing before retained
  facades can be removed or hard-deprecated.
- Final compatibility-facade retention/deprecation evidence is still missing:
  the documented decision is to delete old external-engine runtime
  implementations, keep only native-backed compatibility facades required by
  downstream migration, and leave FCL/Bullet/ODE access in explicit
  reference-only test/benchmark APIs.
- Full local `pixi run test-all` evidence is refreshed at `778cc98c5f2`, with
  normal native-only defaults keeping FCL, Bullet, ODE, reference tests, and
  reference benchmarks `OFF`. Final `pixi run test-all` evidence after the
  eventual PR-complete state is still missing. The refreshed full reruns found
  and repaired three
  local validation robustness gaps: a stale optional `libccd` CMake cache issue
  in the default native-only test build, stale `ShapeType::Cone`,
  `ShapeType::HeightField`, and `ShapeType::PointCloud` cases in the VSG
  geometry builder after those native tags were removed, and an over-broad
  nanobind source-refresh marker in `scripts/test_all.py` that could refresh
  `nb_func.cpp` without rebuilding matching `nb_internals.cpp` objects.
- The dev-task folder must remain until final PR evidence is transferred to
  the PR description and the folder is deleted in the completing PR. Durable
  architecture notes are now seeded in onboarding docs, and
  `07-pr-evidence-transfer.md` stages a PR-template-shaped evidence packet, but
  final evidence transfer is still pending.

## Audit Inputs

Current audited state:

- Branch: `feature/new_coll`
- Latest pushed branch head before this audit-record update: `ad1b6782578`
  (`Align public collision docs with native default`). Local and
  `origin/feature/new_coll` matched at that head, PR #2652 was still closed
  and anchored to old head `714d220d82a`, and GitHub reported no Actions runs
  for commit `ad1b6782578`.
- Latest published checkpoint before CI repair: `5b08a00d381`
  (`Record current native collision validation pass`), which records the
  current local validation pass after the durable built-in architecture docs
  and compatibility-facade policy cleanup.
- Latest audited branch evidence: `ec6f6f43112`
  (`Clean dartpy collision API and deprecate C++ facades`) plus
  `aa3ccce70c7` (`Clarify collision reference build options`) through
  `ad1b6782578` (`Align public collision docs with native default`). Those
  commits include the latest code/evidence head `1e1faf6feb1`
  (`Fix native capsule convex casts in CI`), the local downstream evidence
  checkpoint, the compatibility-facade audit guard at `f31f1a5b897`, the PR
  evidence transfer draft, the closed-PR CI trigger evidence at `6be640e7007`,
  the clean dartpy collision API, default-on C++ compatibility facade
  deprecations, and docs clarifying that `DART_BUILD_COLLISION_FCL`,
  `DART_BUILD_COLLISION_BULLET`, and `DART_BUILD_COLLISION_ODE` only enable
  explicit `collision-reference-*` comparison components. They also include the
  public overview, numerical-methods, constraints, and example README cleanup
  that removes stale normal-runtime FCL/Bullet/ODE backend guidance. GitHub
  reports no Actions runs for the later pushes because `feature/new_coll` does
  not match the workflow `push` filters and PR #2652 remains closed.
- Full local validation run head: `778cc98c5f2`
  (`Clarify collision reference flag scope`). The safe-job full-validation
  command passed with 6/6 top-level gates: linting, build, unit tests,
  simulation-experimental tests, Python tests, and documentation. The configure
  output kept `DART_BUILD_COLLISION_FCL`,
  `DART_BUILD_COLLISION_BULLET`, `DART_BUILD_COLLISION_ODE`, collision
  reference tests, and collision reference benchmarks `OFF`, confirming those
  options are reference-component opt-ins rather than compatibility
  prerequisites for core DART, dartpy, gz-physics runtime integration, or
  native-backed package component facades.
- Benchmark guard run head: `1f3505ea469`
  (`Record latest native collision validation`). The safe-job
  `collision-reference` benchmark guard passed locally, covering narrow phase,
  distance, raycast, mixed primitives, mesh-heavy, raycast-batch, and public
  DART adapter benchmark JSON outputs. This is local benchmark evidence; it
  does not replace final PR/CI artifact evidence because feature-branch pushes
  do not trigger the benchmark workflow while PR #2652 is closed.
- GitHub PR state: PR #2652
  (https://github.com/dartsim/dart/pull/2652) is closed, still marked draft,
  and remains anchored to old head `714d220d82a`.
- Closed-PR workflow-dispatch state for `658da0edb10`:
  - CI Linux: run `25885373625`
    (https://github.com/dartsim/dart/actions/runs/25885373625), in progress
    overall, with `Native Collision (no FCL/Bullet/ODE)` failed at job
    `76075528745` in `test_ccd` / `CapsuleCastConvex.DirectHit`.
  - CI gz-physics: run `25885373580`
    (https://github.com/dartsim/dart/actions/runs/25885373580), passed.
  - Publish dartpy: run `25885373596`
    (https://github.com/dartsim/dart/actions/runs/25885373596), passed across
    all wheel build, repair, verify, test, and upload jobs. Artifact names:
    `ubuntu-latest-py312`, `ubuntu-latest-py313`, `ubuntu-latest-py314`,
    `macos-latest-py312`, `macos-latest-py313`, `macos-latest-py314`,
    `windows-latest-py312`, `windows-latest-py313`,
    `windows-latest-py314`.
  - Pushed repair after the Linux native-only failure: replace
    endpoint-only `capsuleCastConvex()` checks with capsule-support
    conservative advancement against the convex support target. Focused local
    validation passes `test_ccd`, the 29-test `collision-native` label, the
    four native-only CI focused CTest executables, and the 17-test Python
    collision/world smoke.
  - Same-run benchmark guard state: `Collision Benchmark Guard` job
    `76075528749` passed its benchmark command, but uploaded no artifact
    because `actions/upload-artifact@v6` skipped the hidden
    `.benchmark_results/` directory. The current working tree sets
    `include-hidden-files: true` for that upload step, so the next dispatch can
    attach the required `collision_check_*.json` files.
- Closed-PR workflow-dispatch state for repaired head `1e1faf6feb1`:
  - CI Linux: run `25887939088`
    (https://github.com/dartsim/dart/actions/runs/25887939088). Reference
    status: still in progress overall because broad `Release Tests` job
    `76084265182` is in its ASAN tail; do not block local progress on this
    slow reference-only signal.
  - CI Linux `Native Collision (no FCL/Bullet/ODE)`: job `76084265248`,
    passed. This covers the previous `test_ccd` /
    `CapsuleCastConvex.DirectHit` failure on GitHub.
  - CI Linux `Collision Benchmark Guard`: job `76084265196`, passed.
    Uploaded artifact `collision-benchmark-guard-25887939088-1`, id
    `7006005918`, digest
    `sha256:c92c993b9d6a2eaf0ac234d7526cc9893c6544992f0ed55fd77a8bf7f02ba2f5`,
    size `20912` bytes, for head
    `1e1faf6feb18148c1d9fbc041e75a3cb8da457ef`.
  - CI gz-physics: run `25887940214`
    (https://github.com/dartsim/dart/actions/runs/25887940214), passed. The
    GZ Physics Tests job `76084227295` passed.
  - Publish dartpy: run `25887941240`
    (https://github.com/dartsim/dart/actions/runs/25887941240), passed across
    all wheel build, repair, verify, test, and upload jobs for Ubuntu, macOS,
    and Windows on Python 3.12, 3.13, and 3.14. The PyPI publish job was
    skipped as expected for this dispatch.
  - Wheel artifact IDs from the repaired-head run:
    `7006040650` (`ubuntu-latest-py312`), `7006017034`
    (`ubuntu-latest-py313`), `7005950943` (`ubuntu-latest-py314`),
    `7005940232` (`macos-latest-py312`), `7005904083`
    (`macos-latest-py313`), `7005879764` (`macos-latest-py314`),
    `7006075376` (`windows-latest-py312`), `7006153851`
    (`windows-latest-py313`), and `7006180173`
    (`windows-latest-py314`).
- Current pushed-head local validation for `1e1faf6feb1`:
  - `pixi run lint` passed before the commit.
  - `pixi run build` passed after the commit was pushed.
  - `pixi run test-unit` passed after the commit was pushed, reporting
    277/277 tests passed. The run includes `test_ccd` and all 29
    `collision-native` tests.
  - A focused local ASAN attempt is blocked by the local host toolchain, not by
    a DART test failure: `/usr/lib/gcc/x86_64-redhat-linux/15/libasan.so` is a
    linker script pointing at missing `/usr/lib64/libasan.so.8.0.0`, so the
    ASAN build fails at link time before native collision tests can run.
- Clean-API baseline local validation for `ec6f6f43112`:
  - `pixi run lint` passed before the commit and included
    `check-collision-runtime-isolation` plus
    `audit-collision-compat-facades`.
  - `pixi run build` passed.
  - `pixi run test-unit` passed, reporting 277/277 CTest tests passed.
  - `pixi run test-py` passed, reporting 147 Python tests passed and proving
    the clean dartpy `DartCollisionDetector` API without legacy detector
    aliases.
  - `pixi run -e gazebo test-gz` passed 65/65 tests with the DART install
    configured native-only: `DART_BUILD_COLLISION_FCL=OFF`,
    `DART_BUILD_COLLISION_BULLET=OFF`, and
    `DART_BUILD_COLLISION_ODE=OFF`.
  - The native compatibility package smoke passed against that native-only
    install, including retained `collision-fcl`, `collision-bullet`, and
    `collision-ode` package component facades.
  - `gh run list --branch feature/new_coll --commit ec6f6f43112 ...` returned
    no workflow runs because feature-branch pushes are not workflow triggers
    for the main CI surfaces while PR #2652 is closed.
- Build-option policy docs validation for `aa3ccce70c7`:
  - `pixi run lint` passed and configured the default build with
    `DART_BUILD_COLLISION_FCL=OFF`, `DART_BUILD_COLLISION_BULLET=OFF`,
    `DART_BUILD_COLLISION_ODE=OFF`,
    `DART_BUILD_COLLISION_REFERENCE_TESTS=OFF`, and
    `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF`.
  - The lint gate included `check-collision-runtime-isolation` and
    `audit-collision-compat-facades`, whose output still reported
    `dartpy API: DartCollisionDetector only; legacy detector aliases absent`
    and `package components: collision-fcl/bullet/ode -> dart`.
  - `git diff --check` passed before commit.
  - `gh run list --branch feature/new_coll --commit aa3ccce70c7 ...` returned
    no workflow runs because feature-branch pushes are not workflow triggers
    while PR #2652 is closed.
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
    include protected release/default branches. Manual workflow dispatch later
    supplied reference evidence for head `1e1faf6feb1`, but the current user
    direction is commit pushes only, with no PR reopen/create or workflow
    dispatch mutation.

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

- Closed-PR follow-up full local validation:

  ```bash
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all
  ```

  Result: passed after one test-harness repair. The first rerun failed linking
  `dartpy` because `scripts/test_all.py` treated the valid nanobind line
  `buf.put(" = ");` as a corruption marker, refreshed `nb_func.cpp`, and left
  stale `nb_internals.cpp` objects without `internals_inc_ref()` /
  `internals_dec_ref()`. The helper now only matches the invalid escaped
  default-argument line, refreshes `nb_internals.cpp` and `nb_internals.h`
  together with refreshed nanobind sources, and touches internals when
  `nb_func.cpp` is newer. Focused Release and Debug `dartpy` target rebuilds
  passed, then the full suite reported 6/6 top-level gates passed: linting,
  build, unit tests, simulation-experimental tests, Python tests, and
  documentation.

- Current local validation refresh on docs-only head `f5d4f9ee932`:

  ```bash
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all
  ```

  Result: passed. The comprehensive suite reported 6/6 top-level gates passed:
  linting, build, unit tests, simulation-experimental tests, Python tests, and
  documentation. Release CTest reported 264/264 tests passed, including the
  29-test `collision-native` label. Configure output kept the native-only
  defaults with FCL, Bullet, ODE, reference tests, and reference benchmarks all
  `OFF`.

- Local command run during this audit:

  ```bash
  python scripts/check_collision_runtime_isolation.py
  ```

- Observed output:

  ```text
  Collision runtime isolation check passed.
  ```

- Follow-up local compatibility-facade audit hardening:

  ```bash
  python scripts/check_collision_runtime_isolation.py
  python scripts/audit_collision_compat_facades.py
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run lint
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run check-lint
  ```

  Result: passed. The runtime isolation guard now also verifies that top-level
  legacy collision engine files are explicit public facades forwarding only to
  native-backed `compat/` headers. The new
  `audit-collision-compat-facades` Pixi task audits retained factory keys,
  C++ detector/group facades, absence of dartpy legacy detector aliases, and
  CMake legacy component names as native-backed routes, and both
  `pixi run lint` and `pixi run check-lint` execute it with the existing
  isolation guard.

- Final local validation after compatibility-facade audit hardening:

  ```bash
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all
  ```

  Result: passed. The comprehensive suite reported 6/6 top-level gates passed:
  linting, build, unit tests, simulation-experimental tests, Python tests, and
  documentation. The lint gate included `audit-collision-compat-facades` and the
  existing collision runtime-isolation guard.

- Final runtime cleanup reference-file audit:

  ```bash
  find dart/collision/{fcl,bullet,ode}/reference -type f | sort
  rg -n 'file\(GLOB|dart_add_library|add_component_targets' \
    dart/collision/fcl/CMakeLists.txt \
    dart/collision/bullet/CMakeLists.txt \
    dart/collision/ode/CMakeLists.txt
  ```

  Result: inspected all old-engine implementation files under explicit
  `reference/` paths. FCL has 12 files, Bullet has 16, and ODE has 26. Each
  engine CMake file globs the corresponding `reference/*.cpp` and
  `reference/detail/*.cpp` sources where applicable, builds only the matching
  `dart-collision-reference-*` target, and registers only the matching
  `collision-reference-*` package component. No unreferenced old-engine runtime
  file was found outside the explicit reference targets, so this audit deletes
  nothing and preserves the remaining reference implementations for
  `createReference()`, reference tests, and comparative benchmarks.

Additional inspected artifacts:

- `docs/dev_tasks/native_collision/README.md`
- `docs/dev_tasks/native_collision/01-design.md`
- `docs/dev_tasks/native_collision/03-evidence-gates.md`
- `docs/dev_tasks/native_collision/04-reference-gap-analysis.md`
- `docs/dev_tasks/native_collision/05-downstream-migration.md`
- `docs/dev_tasks/native_collision/07-pr-evidence-transfer.md`
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

| Requirement                                                                                                                  | Evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Status  |
| ---------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- |
| Track progress on the scale of the path toward the north star.                                                               | `README.md` and `03-evidence-gates.md` both define the 0-12 north-star progress scale. Stage 10 has local design evidence, stage 11 has local source/reference split evidence, and stage 12 remains blocked on CI, migration, final deletion, validation, and PR packaging.                                                                                                                                                                                                    | Done    |
| One PR continues until it fully reaches the north star.                                                                      | `README.md` states the checkpoint is not a final PR boundary, and `02-milestones.md` keeps final PR packaging and dev-task deletion as completion gates.                                                                                                                                                                                                                                                                                                                       | Open    |
| `dart/collision/` stops exposing a real multi-backend runtime selection layer.                                               | `dart/collision/dart/dart_collision_detector.hpp` owns the canonical `dart` registrar plus legacy alias registrars. Top-level FCL/Bullet/ODE headers route through compatibility facades, and old implementation sources live under explicit `reference/` paths.                                                                                                                                                                                                               | Local   |
| Retained FCL/Bullet/ODE/experimental names are wrappers/adapters over the built-in detector.                                 | Compatibility headers under `dart/collision/{fcl,bullet,ode}/compat/` and the package smoke under `docs/dev_tasks/native_collision/smoke/native_compat_package/` verify native-backed C++ and package compatibility names. Dartpy uses the clean `DartCollisionDetector` API and tests/audit verify legacy detector aliases are absent.                                                                                                                                        | Local   |
| Any selected "backend" through retained public names always uses the built-in detector.                                      | `UNIT_collision_DartCollisionDetector` covers factory aliases and legacy facade behavior. The native compatibility package smoke verifies package components, factory keys, installed headers, and direct legacy `create()` calls.                                                                                                                                                                                                                                             | Local   |
| FCL/Bullet/ODE remain only as optional reference engines for tests and benchmarks.                                           | `DART_BUILD_COLLISION_REFERENCE_TESTS` and `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` exist in CMake/Pixi configure paths; `collision-reference` is the explicit opt-in Pixi environment; `DART_BUILD_COLLISION_FCL/BULLET/ODE` now build only optional reference components and are not needed by core DART, dartpy, gz-physics runtime integration, or native-backed compatibility facades; reference call sites use `createReference()` and `collision-reference-*` names. | Local   |
| Native-only builds can opt out of FCL/Bullet/ODE.                                                                            | Normal Pixi configure paths default FCL, Bullet, ODE, reference tests, and reference benchmarks to `OFF`; local native-only build/install/wheel evidence is recorded in `03-evidence-gates.md`.                                                                                                                                                                                                                                                                                | Local   |
| Default packages and wheels do not carry old collision runtime dependencies.                                                 | Package install probes, Pixi dependency metadata checks, and the `verify_wheel_collision_isolation.py` verifier are recorded. `wheel-verify-core` runs the wheel isolation verifier, and `publish_dartpy.yml` invokes `pixi run -e py${{ matrix.python-version }}-wheel wheel-verify`.                                                                                                                                                                                         | Local   |
| Built-in detector maintains correctness tests.                                                                               | Native unit/integration tests, feature parity tests, DART adapter tests, gz-focused regressions, and reference comparison tests are recorded in `03-evidence-gates.md`.                                                                                                                                                                                                                                                                                                        | Local   |
| Built-in detector maintains benchmarks for performance optimization.                                                         | `pixi run -e collision-reference bm-collision-check` runs checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy, raycast-batch, and public adapter benchmark subsets, writing `.benchmark_results/collision_check_*.json`.                                                                                                                                                                                                                                       | Local   |
| Native performance should beat the best legacy engine on required workloads, with feature first and correctness preserved.   | The current local broad benchmark guard passed at `1f3505ea469`; `README.md` records native wins on the measured primitive, narrowphase, supported distance, raycast, batch, mesh-heavy, and mixed workloads.                                                                                                                                                                                                                                                                  | Local   |
| Built-in layer architecture must be API-clean, scalable, and performance-oriented.                                           | `01-design.md` defines the public API boundary, compatibility shell, DART adapter scene, native scene/query core, query lifecycle, scalability design, performance hooks, and reference harness boundary. `README.md` mirrors this in the Architecture Completion Rubric, `docs/onboarding/architecture.md` now carries the durable built-in runtime architecture summary, and native `ShapeType` cleanup keeps the native taxonomy aligned with real supported classes.       | Local   |
| Architecture evidence must cover code, tests, package/source boundaries, and benchmark/profiling hooks, not only prose docs. | Code evidence exists for factory aliases, compatibility facades, persistent `DartCollisionGroup` scene state, cache IDs, source isolation, native shape taxonomy cleanup, wheel verifier wiring, package smoke, adapter benchmarks, native/reference benchmark JSON, repaired-head workflow-dispatch wheel artifacts, and the benchmark guard artifact. Final PR-state CI/artifact evidence remains missing.                                                                   | Partial |
| gz-physics compatibility must be preserved while legacy names migrate.                                                       | Fresh local `pixi run -e gazebo test-gz` passed 65/65 and package smoke passed. `readelf` shows the gz DART plugin and package-smoke executable use `libdart-collision-native.so` without old collision/reference runtime dependencies. `05-downstream-migration.md` defines the compatibility contract and removal gates.                                                                                                                                                     | Local   |
| Downstream migration/deprecation path must be proven before removing retained facades.                                       | `05-downstream-migration.md` defines the migration order and gates. Local primary evidence now covers fresh gz-physics, package smoke, runtime isolation, C++ compatibility names, dartpy clean-API enforcement, direct link inspection, and default-on C++ deprecation annotations. Final pushed-head CI/PR evidence remains open.                                                                                                                                            | Open    |
| Final PR evidence and cleanup must happen in the same PR.                                                                    | `README.md`, `02-milestones.md`, and `docs/dev_tasks/README.md` require transferring final evidence to the PR description and deleting `docs/dev_tasks/native_collision/` only at completion. Durable collision architecture notes have been seeded in onboarding docs, and `07-pr-evidence-transfer.md` stages the PR-template-shaped evidence packet, but PR evidence transfer and folder deletion remain open.                                                              | Open    |

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
   state but does not automatically trigger the main GitHub Actions workflows.
   Manual workflow-dispatch reference evidence for pushed head `1e1faf6feb1`
   now covers native-only CI, gz-physics, the wheel matrix, and the collision
   benchmark guard artifact upload. Later pushed docs/evidence heads have no
   attached runs for the same trigger-filter reason.
   Treat GitHub CI as reference evidence; use local build/test as the main
   validation surface unless the maintainer explicitly chooses another trigger.
2. Record downstream deprecation policy evidence proving downstream users no
   longer depend on legacy names as runtime backend selectors before removing
   or hard-deprecating retained facades. Local migration evidence is refreshed;
   the remaining gap is policy/finalization, not the package/gz smoke itself.
3. Apply the documented compatibility-facade policy in the final PR state:
   preserve only wrappers required for source compatibility, keep them
   native-backed, and keep all external engines reference-only. The local
   reference-file cleanup audit found no unreferenced old-engine implementation
   files to delete; the remaining FCL/Bullet/ODE implementation files are
   intentional `collision-reference-*` test/benchmark code.
4. Run final validation after the final code state, including at least
   `pixi run lint` and `pixi run test-all`, plus any CI-specific gates whose
   failures are not covered locally.
5. Keep the durable collision architecture summary in onboarding docs, use
   `07-pr-evidence-transfer.md` as the starting packet for the final PR
   description, then delete this dev-task folder in the same PR.

## Completion Bar

Do not mark this dev task complete until every row in the checklist is either
`Done` with durable docs evidence or backed by CI/PR artifacts that close the
remaining `Local`, `Partial`, and `Open` statuses. Until then,
`docs/dev_tasks/native_collision/` remains active working documentation.
