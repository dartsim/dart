# Native Collision North-Star PR - Dev Task

## Current Scope

The active user scope is branch-local feature/evidence completion. Do not open
or reopen a PR, trigger GitHub workflows, mutate PR metadata, or delete this
folder in the current pass. Keep `feature/new_coll`, `PR-DRAFT.md`, and
`07-pr-evidence-transfer.md` ready for the maintainer-opened review surface.
Final PR CI evidence, evidence transfer, and folder deletion are deferred
finalization steps.

Latest local follow-up: the current tree fixes the default native box-ground
contact regression reported from `hello_world`/Atlas-style scenes, makes
invalid convex/soft mesh data non-collidable with a warning, adds focused raw
box-box, sphere-sphere batch, capsule-capsule batch, cylinder-cylinder batch,
convex-mesh, mesh, default-world, and Atlas foot-ground regression coverage,
rebuilds `hello_world` without the OctoMap `<ciso646>` warning, and refreshes
the focused native/reference/benchmark validation evidence in
`03-evidence-gates.md`.

## Current Status

- [x] Native collision core exists under `dart/collision/native/`.
- [x] DART adapter exists under `dart/collision/dart/`.
- [x] Legacy `"experimental"` factory key is retained as a compatibility alias.
- [x] Retained legacy factory keys `"fcl"`, `"fcl_mesh"`, `"bullet"`, and
      `"ode"` now resolve to the built-in `DartCollisionDetector` in both
      native-only and reference-enabled builds.
- [x] `dart` is the first-choice default detector in core world, constraint,
      and SKEL-loading paths.
- [x] Native coverage is proven against the DART feature surface currently
      served by FCL, Bullet, and ODE, including the final `VoxelGridShape`
      parity gap found by audit.
- [x] Full local gz-physics compatibility is proven with
      `pixi run -e gazebo test-gz` from a fresh downstream clone: 65/65 tests
      passed. The local fixes cover DART/gz custom mesh-plane free-fall through
      collision-object-order contact normals, stacked cylinder support through
      parallel-cylinder cap/side contact selection, axial and tilted
      cylinder-vs-plane-like-box support for gz's plane-as-large-box path,
      capped large flat box/mesh contact patches for gz max-contact tests, and
      legacy FCL/ODE facade raycast behavior required by gz's ray-intersection
      feature tests. The latest local refresh on code head `64abc65a032`
      rebuilt a fresh gz-physics checkout, printed the expected DART plugin
      integration success line, and `readelf` showed the gz DART plugin
      depends on `libdart-collision-native.so` without any
      `libdart-collision-reference-*`, `libdart-test-reference-*`, FCL, Bullet,
      ODE, or libccd runtime dependency. Manual workflow-dispatch evidence on
      `1e1faf6feb1` is
      reference-only and also passed gz-physics CI, but final PR packaging
      still needs maintainer-approved evidence transfer because PR #2652 is
      closed.
- [x] Comparative benchmarks prove native is at least as fast as the best
      legacy backend for required workloads. Primitive, narrow-phase,
      supported distance, raycast, raycast-batch, mesh-heavy, and
      mixed-primitive scenario cases pass. The previous mixed-primitives dense
      1000 loss is now a win through cached broad-phase snapshot reuse:
      native is 1.06 ms CPU mean versus Bullet at 1.69 ms.
- [x] FCL, Bullet, and ODE are no longer required collision dependencies. A
      core `dart` build, focused native/default C++ tests, and `dartpy` now
      build without exposing or requiring per-engine collision build switches.
      Public source-build prerequisites now keep FCL with Bullet and ODE as
      optional reference-comparison dependencies rather than required core
      runtime dependencies.
- [x] Normal pixi configure paths now default reference correctness tests and
      reference benchmarks to `OFF`; comparison jobs opt in through the
      `collision-reference` environment, which enables
      `DART_BUILD_COLLISION_REFERENCE_TESTS` and
      `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS`.
- [x] Per-engine FCL/Bullet/ODE collision build switches are gone from the
      current build surface. Core DART, dartpy, gz-physics runtime integration,
      and the native-backed `collision-fcl`/`collision-bullet`/`collision-ode`
      compatibility facades no longer need backend-shaped build flags.
- [x] Human-authored user docs no longer describe FCL, Bullet, or ODE as normal
      runtime collision backends. The public overview, numerical-methods,
      constraints, and example docs describe the built-in detector as the
      runtime path and the old engines as optional reference-comparison inputs.
- [x] Native-only install metadata no longer advertises old collision runtime
      libraries; retained legacy collision component targets are native-backed
      interface facades, and the old per-engine build option variables are no
      longer part of the installed metadata.
- [x] A fresh native-only runtime install probe with `dartpy`, GUI, VSG GUI, IO,
      utils, URDF, simulation-experimental, and `dart-collision-native` built
      and installed shows no old collision component files and no FCL, Bullet,
      ODE, or libccd runtime links.
- [x] Default and wheel Pixi dependency metadata no longer carry FCL, Bullet,
      ODE, or their FCL transitive packages. The new `collision-reference`
      environment is the explicit opt-in path for reference-engine correctness
      tests and benchmarks.
- [x] A repaired `py312-wheel` artifact built with old collision engines and
      reference harnesses disabled imports successfully, exposes the expected
      dartpy modules, and contains no old collision component headers, old
      collision component libraries, old collision CMake exports, or FCL,
      Bullet, ODE, or libccd runtime links.
- [x] `wheel-verify` now includes a collision-isolation check that rejects old
      runtime libraries, reference collision libraries, and reference component
      exports while allowing native-backed FCL/Bullet/ODE compatibility
      component facades in dartpy wheel artifacts. The existing repaired py312
      Linux wheel passes that check locally.
- [x] `dartpy` no longer links legacy collision component targets and keeps a
      clean DART 7 Python collision API: `DartCollisionDetector` is the
      detector class, while legacy aliases `DARTCollisionDetector`,
      `FCLCollisionDetector`, `BulletCollisionDetector`, and
      `OdeCollisionDetector` are not exposed.
- [x] The `collision-reference` environment configures with reference tests and
      reference benchmarks enabled, derives all FCL, Bullet, and ODE reference
      components internally, and the focused `test_reference_backends` target
      builds and passes.
- [x] C++ reference-engine call sites in tests and benchmarks now use explicit
      `FCLCollisionDetector::createReference()`,
      `BulletCollisionDetector::createReference()`, and
      `OdeCollisionDetector::createReference()` APIs. Focused reference
      unit/integration targets, comparative benchmark targets, CTest, and
      benchmark smoke runs pass with those APIs.
- [x] Direct public C++ legacy detector `create()` entry points are
      native-backed `DartCollisionDetector` facades. They may preserve legacy
      display type strings for gz-physics compatibility, but they do not
      instantiate FCL, Bullet, or ODE. The old FCL, Bullet, and ODE engines
      remain reachable only through explicit `createReference()` APIs in the
      reference-enabled component builds.
- [x] User-facing examples and tutorials no longer require or link old
      collision component targets. Examples/tutorials that previously selected
      Bullet or ODE now use the built-in detector/default collision path;
      legacy engine dependencies are kept to reference tests and benchmarks.
- [x] Optional old-engine CMake targets are now test-only and explicitly
      reference-named: `dart-test-reference-fcl`,
      `dart-test-reference-bullet`, and `dart-test-reference-ode`. The old
      installable `collision-reference-*` package components are gone.
- [x] Retained legacy package component names `collision-fcl`,
      `collision-bullet`, and `collision-ode` are native-backed interface
      facades. A default native-only install/export probe and downstream
      package smoke test that requests `collision-fcl`, `collision-bullet`, and
      `collision-ode` proves those names link the built-in `dart` stack and do
      not install old collision libraries. The latest local refresh on code
      head `64abc65a032` reran that package smoke and `readelf` showed the
      smoke executable depends on `libdart-collision-native.so` without any
      old collision/reference runtime dependency.
- [x] Installed legacy detector headers for FCL, Bullet, and ODE are now
      compatibility facades over `DartCollisionDetector` in native-only and
      reference-enabled installs. The downstream package/header smokes include
      the installed legacy detector headers and verify factory keys plus legacy
      class `create()` calls all report detector type `dart`.
- [x] `05-downstream-migration.md` defines the downstream compatibility
      contract: retained C++ legacy detector names, factory keys, and package
      components are source-compatible routes to the built-in detector, dartpy
      uses the clean `DartCollisionDetector` API without legacy Python aliases,
      reference engines require explicit `dart-test-reference-*` targets plus
      `createReference()` APIs, and the deprecation evidence acceptance
      checklist names the artifacts that close the downstream policy gate.
- [x] Public migration docs now carry the same DART 7 policy: C++ legacy
      collision names are native-backed migration facades, dartpy does not keep
      old detector aliases, and FCL/Bullet/ODE comparison engines are opt-in
      test-only reference targets rather than normal runtime build options.
- [x] Source-tree legacy detector headers are now split the same way as the
      installed package surface: top-level FCL, Bullet, and ODE detector/group
      headers, PascalCase compatibility headers, All headers, and component
      headers are native-backed compatibility facades, while old-engine
      implementation headers and sources live under
      `tests/dart/test/reference_collision/` and are used only by reference
      tests and benchmarks.
- [x] A runtime source isolation check is wired into `lint` and `check-lint`.
      It fails if runtime source paths include FCL, Bullet, ODE, libccd, or old
      collision reference backend headers, if non-test code includes
      `dart/test/reference_collision/...`, or if legacy engine implementation
      sources move back under runtime collision directories.
- [x] A broad recurring benchmark guard exists:
      `pixi run -e collision-reference bm-collision-check` builds and runs
      checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy,
      raycast-batch, and public DART adapter benchmark subsets. Native/reference
      subsets emit Google Benchmark JSON and compare native timings against the
      best enabled FCL/Bullet/ODE reference result; the public adapter subset
      records JSON for the built-in `DartCollisionDetector` path. The broad
      local guard was refreshed at `4b155655890` and passed every checked
      subset after the latest validation and benchmark-evidence updates.
- [x] CI Linux contains a scheduled/manual collision benchmark guard job that
      runs the broad `collision-reference` benchmark check and uploads the
      `.benchmark_results/collision_check_*.json` artifacts when a workflow
      surface is available.
- [x] Recent local benchmark-guard evidence is refreshed. The
      `collision-reference` benchmark guard passed before the `4b155655890`
      benchmark-evidence commit, producing local
      `.benchmark_results/collision_check_{narrow,distance,raycast,mixed,mesh,raycast_batch,adapter}.json`
      files.
- [ ] Final benchmark-guard CI artifact evidence on the completing PR head is
      still open. The last manual GitHub artifact evidence is tied to
      `1e1faf6feb1`; later feature-branch pushes do not start GitHub Actions
      while PR #2652 is closed.
- [ ] Final PR/CI review surface evidence is still open. The implementation is
      locally validated and prior manual workflow-dispatch runs are reference
      evidence, but the final review surface must be maintainer-selected
      because PR #2652 is closed.
- [ ] Downstream migration/deprecation finalization is still open. The local
      contract, C++ deprecation option, clean dartpy API, acceptance criteria,
      gz-physics smoke, package smoke, and link inspections exist; final
      downstream/PR evidence still has to prove retained names are migration
      facades rather than runtime backend selectors.
- [ ] Final compatibility-facade/runtime cleanup policy is still open. The
      current reference-file audit found no unreferenced old-engine
      implementation files to delete; remaining FCL/Bullet/ODE code is
      intentional `dart-test-reference-*` test/benchmark code.
- [ ] Final validation after the completing code state is still open. At
      minimum this means `pixi run lint`, `pixi run test-all`, and any
      maintainer-selected CI gates whose failures are not covered locally. The
      latest full local `pixi run test-all` pass was rerun on the current
      working tree after pushed head `f8f5663d514` and passed 6/6 top-level
      gates with Release CTest 264/264 and Python tests 147/147. The current
      clean reference-gate refresh proves normal configure exposes only
      reference tests/benchmarks, keeps both `OFF` by default, and no longer
      has per-engine FCL/Bullet/ODE collision build switches. The
      source-build prerequisite cleanup was linted on pushed docs commit
      `621fca5a1fb`; final maintainer-selected PR/CI evidence is still
      pending.
- [ ] Final evidence transfer and dev-task cleanup are still open.
      `07-pr-evidence-transfer.md` and `PR-DRAFT.md` stage the review packet,
      but this folder must stay until that evidence is moved to the completing
      PR and the folder is deleted in the same PR.

## Goal

Make DART's built-in/native collision detector the default implementation and
the long-term replacement for FCL, Bullet, and ODE collision backends. The
replacement must be evidence driven: feature coverage, behavior compatibility,
gz-physics compatibility, native-only package/wheel isolation, and benchmark
guardrails must be verified before dependency removal.

Current scope: this pass is feature-level completion. It must leave
performance measurement hooks, benchmark baselines, and regression guardrails in
place, but the next wave owns performance optimization: first single-CPU hot
paths, then multi-core CPU parallelism, with GPU support as a stretch goal.

## North Star

DART should not require FCL, Bullet, or ODE for collision detection. The native
detector should cover the DART collision feature surface, preserve gz-physics
compatibility, and expose benchmark/profiling surfaces for future optimization.
`dart/collision/` should expose one built-in collision detector stack, not a
real multi-backend selection layer. Any legacy detector name, header, factory
key, or component retained for gz-physics compatibility must be a thin adapter
over the built-in detector.

The built-in detector must also have a clean component architecture: public
DART collision APIs and compatibility facades at the outside, a DART adapter
layer for shape/result/filter conversion, and a native scene/query core for
geometry, broadphase, narrowphase, distance, raycast, contact persistence,
caching, and profiling. Public APIs should expose DART semantics rather than
engine-specific knobs, while the native core stays scalable and
performance-oriented internally.

FCL, Bullet, and ODE may remain only as optional reference engines for
correctness tests and benchmarks. They should be controlled by the public
reference test/benchmark gates so comparison jobs can opt in and native-only
builds stay free of per-engine runtime switches. The work order is feature
coverage first, correctness tests as the permanent gate, then a follow-up
performance wave that uses the retained benchmark guardrails to optimize
single-CPU, multi-core CPU, and possible GPU paths.

## North-Star Progress Scale

This task is scoped as one PR that continues until the north star is reached.
The current checkpoint is a validated middle state, not a final PR boundary.

| Stage | Progress marker                              | Status                                                                                        |
| ----- | -------------------------------------------- | --------------------------------------------------------------------------------------------- |
| 0     | Baseline native backend exists               | Complete before this task                                                                     |
| 1     | Native `dart` detector is the default path   | Local complete                                                                                |
| 2     | DART feature parity gaps are closed          | Local complete                                                                                |
| 3     | gz-physics compatibility is proven           | Local complete; final PR/downstream CI evidence open                                          |
| 4     | Benchmark guardrails exist for performance   | Local baseline complete; final PR benchmark artifact evidence open; optimization is next wave |
| 5     | FCL/Bullet/ODE are optional for local builds | Local complete                                                                                |
| 6     | Native-only and gz-physics CI are permanent  | Manual workflow-dispatch reference evidence exists; final PR CI open                          |
| 7     | Reference engines are test/bench-only        | Local complete; final reference-gate CI evidence open                                         |
| 8     | Default packages have no old runtime deps    | Manual wheel-matrix reference evidence exists; final PR wheel CI open                         |
| 9     | Downstream migration/deprecation path exists | Local package/gz/link evidence exists; final PR/downstream CI open                            |
| 10    | Clean built-in API/scaling/perf layer        | Local design/artifact evidence exists; final PR CI/artifact open                              |
| 11    | Old runtime backend source is reference-only | Local split and lint guard complete; final PR-source audit evidence open                      |
| 12    | Final one-PR validation and PR packaging     | Local validation exists; final PR evidence transfer and folder deletion open                  |

## Built-In Architecture Status

`01-design.md` is the architecture and design contract for the built-in
collision component. This is a feature-level completion gate, not background
context: the PR is not done until the runtime layer is API-clean, scalable, and
performance-ready in code and evidence. The target layer shape is public
DART collision APIs and temporary compatibility facades at the outside,
`dart/collision/dart/` as the DART shape/result/filter adapter, and
`dart/collision/native/` as the scalable scene/query core for geometry,
broadphase, narrowphase, distance, raycast, contact/manifold caching,
deterministic results, profiling, and benchmarks. Its "Architecture
North-Star Requirement" and "Built-In Component Blueprint" are the concrete
design checklists for API cleanliness, adapter-owned scene state, native query
lifecycle, scalability, and performance-oriented internals.

The implementation now satisfies the source/package structure of that design:
public factory keys, retained C++ compatibility names, retained package
components, installed legacy headers, source-tree legacy
detector/group/All/PascalCase headers, and direct legacy `create()` entry
points route to the built-in `dart` detector. Dartpy exposes only the clean
`DartCollisionDetector` detector class for DART 7. Old FCL/Bullet/ODE
implementation headers and sources live under
`tests/dart/test/reference_collision/` and build only through test-only
reference targets. The
`check-collision-runtime-isolation` task now guards the runtime
source tree against reintroducing direct old-engine includes or non-reference
implementation sources. The native core now also documents and tests a
solver-facing result contract: contacts are reported in public collision pair
order, and canonical narrowphase functions are wrapped with explicit normal
flips when dispatch order differs from collision object order. The working
tree also adds the solver cache bridge part of that contract: native manifold
cache writeback follows the attached native `CachedContact`, not the detector
display string preserved by a compatibility facade. Native warm-start contacts
are now also stored in the canonical scene-cache ID order used by the
manifold key, so querying the same pair as A/B and B/A preserves refresh and
matching semantics. The native `ShapeType` taxonomy now also names only real
native shape classes; DART cone, heightmap, point-cloud, and other
approximated or intentionally non-collidable shapes are represented by the
adapter's explicit convex, mesh, compound, or non-collidable behavior instead
of phantom native shape tags. The architecture gate remains open until alias
cleanup, package cleanup, and native algorithms are backed by design evidence
in three dimensions:

- API cleanliness: ordinary users see one `dart` detector surface, semantic
  DART query options/results, native-backed compatibility names, and explicit
  reference-only APIs for old engines.
- Scalability: public collision, distance, and raycast queries use
  adapter-owned persistent scene state, stable native IDs, dirty sync,
  deterministic result assembly, and deliberate cache invalidation.
- Performance orientation: hot paths use compact native geometry,
  shape-specialized dispatch, persistent broadphase/query state, reusable
  scratch/cache lifetimes, and benchmark/profiler labels for each query stage.

That gate remains open for CI evidence, downstream migration/deprecation
evidence, final runtime cleanup, and broader correctness/benchmark guardrails
across the public DART adapter and native core paths. Single-CPU hot-path
optimization, multi-core CPU parallelism, and stretch GPU support are tracked as
the next performance wave rather than current feature-completion blockers.

## Design Readiness Tracker

The status below is measured against the north star, not against whether a
single checkpoint built locally.

| Design axis             | North-star bar                                                                                                                                                                                                                                                             | Current state                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Component layering      | Public DART APIs and compatibility facades sit outside `dart/collision/dart/`; the DART adapter owns scene synchronization and result conversion; `dart/collision/native/` owns algorithms, broadphase, query state, caches, and profiling.                                | The source/package split now matches this shape: legacy public paths are native-backed facades, reference implementation files are under `tests/dart/test/reference_collision/`, and lint guards runtime source isolation. CI/downstream and final deletion evidence remain before the layer is final.                                                                                                                                                                                  |
| API cleanliness         | `dart` is the canonical public detector; legacy keys, C++ classes, headers, and package components are compatibility facades only; dartpy exposes the clean `DartCollisionDetector` API; public options/results describe DART semantics instead of backend-specific modes. | Factory aliases, public C++ legacy `create()` paths, installed headers, source-tree top-level legacy headers, examples, and retained package components are native-backed; dartpy legacy detector aliases are absent. CI/downstream migration evidence is still needed before this is final.                                                                                                                                                                                            |
| Scalability             | Public collision, distance, and raycast use persistent adapter scene state with stable IDs, dirty transform/shape sync, reusable broadphase/query data, cache invalidation, deterministic ordering, and contact results that are stable under pair-order changes.          | Persistent `DartCollisionGroup` scene state, broadphase-pruned raycast, AABB-pruned distance, native filter adaptation, dynamic-shape invalidation coverage, scene-issued manifold cache IDs, and pair-order normal tests are implemented locally. Broader CI and recurring benchmark evidence remain.                                                                                                                                                                                  |
| Performance orientation | Native hot paths use compact geometry, shape-specialized dispatch, persistent broadphase data, reusable scratch, clear cache lifetimes, and profiling/benchmark labels for each query stage.                                                                               | Recorded benchmarks provide feature-level baselines across primitive, narrowphase, supported distance, raycast, batch, mesh-heavy, and mixed-primitive scenarios. The native dispatcher keeps canonical shape-specialized functions while wrapping only result-normal orientation when needed. The recurring benchmark guard covers checked native-vs-reference and public adapter scenarios; deeper single-CPU, multi-core CPU, and stretch GPU optimization is the next wave.         |
| Reference isolation     | FCL, Bullet, and ODE exist only as optional reference engines for tests and benchmarks, with native-only builds able to opt out.                                                                                                                                           | CMake opt-out options, native-only Pixi defaults, explicit `collision-reference` opt-in, test-only `dart-test-reference-*` targets, `tests/dart/test/reference_collision/` source split, runtime source isolation linting, package/wheel metadata cleanup, local install/wheel evidence, wheel artifact verifier wiring, and repaired-head full workflow-dispatch wheel-matrix evidence are in place. Final PR-state packaging evidence remains.                                        |
| Compatibility           | gz-physics and downstream source-compatible C++/package legacy names keep building during migration, but cannot select an external runtime engine. Dartpy uses the clean DART 7 API instead of retaining detector aliases.                                                 | Legacy detector headers/classes, factory aliases, retained package components, and the documented migration contract route to native. Direct legacy facade display strings are covered by DART tests; dartpy alias absence is covered by Python tests/audit. A fresh local `pixi run -e gazebo test-gz` passes 65/65 tests, and `readelf` shows the gz DART plugin and downstream package smoke use `libdart-collision-native.so` without old collision/reference runtime dependencies. |

## Architecture Completion Rubric

Stage 10 on the north-star scale is complete only when the built-in collision
component is clean as an API, scalable as a scene/query system, and ready for
performance work without reopening public backend selection.

| Design gate             | Completion bar                                                                                                                                                                                               | Current evidence                                                                                                                                                                                                                                                                                   | Still needed                                                                                               |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| API cleanliness         | Public DART APIs expose one canonical `dart` detector; legacy C++/package names are native-backed facades; dartpy exposes clean `DartCollisionDetector`; reference engines use explicit test/benchmark APIs. | Factory aliases, C++ legacy `create()` paths, installed/source headers, and package facades route native; dartpy legacy detector aliases are absent.                                                                                                                                               | CI/package matrix evidence and final search proving no public path can select an external runtime backend. |
| Scalable adapter/core   | `DartCollisionGroup` owns persistent scene state, stable IDs, dirty sync, cache invalidation, deterministic results, and reusable query snapshots.                                                           | Public adapter tests and benchmarks cover dirty sync, dynamic geometry invalidation, filters, pair-order normals, cache IDs, the reduced gz-like tilted cylinder support fixture, and the capped large flat box/mesh contact path.                                                                 | Broaden recurring public-adapter correctness evidence and collect CI evidence.                             |
| Performance orientation | Native hot paths use compact geometry, shape-specialized dispatch, persistent broadphase data, reusable scratch/caches, and measured query stages.                                                           | Native-vs-reference benchmark guardrails pass locally across primitive, distance, raycast, batch, mesh-heavy, and mixed workloads.                                                                                                                                                                 | Final PR-state benchmark artifact evidence for the guard; deeper optimization after feature completion.    |
| Reference isolation     | FCL, Bullet, and ODE are optional reference engines only; native-only builds, installs, wheels, and downstream facades do not link them.                                                                     | CMake opt-out, test-only reference targets, lint source isolation, install/package/wheel checks, current package smoke, package-smoke `readelf`, and wheel verifier wiring exist.                                                                                                                  | Final wheel/package evidence and downstream deprecation policy evidence.                                   |
| Compatibility facade    | gz-physics-required spellings compile and run through the built-in detector while migration removes reliance on legacy names.                                                                                | DART facade tests pass, FCL/ODE legacy facades keep gz-required unsupported raycast behavior, current package smoke verifies native-backed component/header facades, fresh local `pixi run -e gazebo test-gz` passes 65/65 tests, and gz/plugin `readelf` shows only the native collision library. | Downstream deprecation policy evidence and final PR evidence transfer.                                     |

## Final CI Closure Map

Use this map after the maintainer opens the successor PR or explicitly chooses
another workflow surface. Feature-branch pushes to `feature/new_coll` do not
start these workflows while PR #2652 is closed.

| Open gate                    | Workflow / job evidence                                                                                                                                     | Notes                                                                                                                              |
| ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| Lint and docs                | `CI Lint` / `Lint`; `CI Lint` / `Documentation`                                                                                                             | Confirms `check-lint`, docs build, runtime isolation, and compatibility facade audits on review head.                              |
| Native-only runtime          | `CI Linux` / `Native Collision (no FCL/Bullet/ODE)`; `CI Linux` / `Release Tests`; `CI Linux` / `Debug Tests`; `CI Linux` / `Asserts enabled (no -DNDEBUG)` | Confirms normal builds keep reference gates off and old engines out of runtime paths.                                              |
| Collision benchmark artifact | `CI Linux` / `Collision Benchmark Guard`                                                                                                                    | This job runs on schedule or manual dispatch; collect the uploaded `collision-benchmark-guard-*` JSON artifact for final evidence. |
| gz-physics downstream        | `CI gz-physics` / `GZ Physics Tests`                                                                                                                        | Confirms downstream runtime compatibility on the review head.                                                                      |
| dartpy wheels                | `Publish dartpy` / `Wheels \| ${{ matrix.os }} Py${{ matrix.python-version }}`                                                                              | Confirms wheel build, repair, verify, test, upload, and collision isolation across the wheel matrix.                               |
| Platform coverage            | `CI macOS` / `Release Tests (arm64)`; `CI macOS` / `Debug Tests (arm64)`; `CI Windows` / `Tests (Release)`                                                  | Confirms the platform-specific repairs still hold on review head.                                                                  |

## Architecture Review Targets

`01-design.md` breaks the built-in collision layer into reviewable
subcomponents: detector/factory surface, compatibility facades,
`DartCollisionGroup` adapter scene, native scene and geometry tables,
broadphase/query snapshots, narrowphase/distance/raycast dispatch, result
builder plus solver cache bridge, and the optional reference harness. Each
subcomponent has an API-cleanliness role, a scalability role, and a
performance role. Progress on Phase 11 should be reported against those
subcomponents, not only against whether old backend names still compile.

## Where To Check Progress

- `README.md`: live north-star status, progress scale, immediate next steps,
  and remaining single-PR epics.
- `01-design.md`: canonical built-in collision architecture and design review
  gates for API cleanliness, scalability, performance, reference isolation, and
  compatibility facades.
- `02-milestones.md`: phase-by-phase success criteria and verification gates.
- `03-evidence-gates.md`: command evidence, observed outputs, and the same
  north-star progress scale tied to validation runs.
- `04-reference-gap-analysis.md`: detailed capability gaps, implementation
  order, and ready-to-implement checklist.
- `05-downstream-migration.md`: compatibility contract and migration/removal
  gates for downstream projects such as gz-physics.
- `06-completion-audit.md`: prompt-to-artifact checklist for deciding whether
  the north star is actually achieved and what evidence is still missing.
- `RESUME.md`: compact handoff state for continuing the same PR without losing
  the latest evidence.

## Non-Goals For This Tracking Task

- Redesigning the constraint solver beyond native collision integration hooks.
- Removing public legacy class names before gz-physics and downstream ABI needs
  are resolved.
- Keeping reference backends in the default build after they stop being needed
  for benchmarks and compatibility validation.

## Key Decisions

- The public default detector key is `dart`; `experimental` remains a temporary
  factory alias only for old saved worlds, tests, and downstream code.
- `dart/collision/native/` owns backend-independent algorithms and data
  structures; `dart/collision/dart/` owns DART API adaptation.
- Reference engines are validation tools during migration, not permanent
  runtime dependencies.
- gz-physics compatibility is a release gate. If gz-physics subclasses or names
  legacy detector classes, DART keeps facade types until gz-physics has a
  compatible path. Those facades must route to the built-in detector regardless
  of the legacy backend name selected through them.
- The completed PR should not leave a real FCL/Bullet/ODE backend abstraction in
  `dart/collision/`. Compatibility names may remain, but they are wrappers or
  adapters over native behavior.
- The built-in collision layer is the architecture boundary: public DART API and
  compatibility facades stay clean, `dart/collision/dart/` adapts DART shapes
  and results, and `dart/collision/native/` owns scalable scene state,
  broadphase, narrowphase, query algorithms, caching, and profiling.
- `01-design.md` is the canonical design contract for the built-in collision
  component. Phase 11 cannot complete until its API-cleanliness, scalability,
  performance, reference-isolation, and compatibility-facade gates are all
  satisfied by code and evidence.
- FCL, Bullet, and ODE may remain only as optional reference engines for tests
  and benchmarks. Native-only builds must be able to opt out through CMake, and
  no default runtime target may depend on them.
- Performance claims require benchmark output checked into this task or linked
  from CI artifacts. Correctness and feature coverage are prerequisites; after
  that, performance work can gradually optimize native until it beats the
  reference engines on required workloads.

## Immediate Next Steps

1. Run and harden the new native-only CI job alongside existing gz-physics CI.
   Local gz-physics evidence is now the full fresh `pixi run -e gazebo test-gz`
   pass, so the next gate is CI visibility for that same downstream build.
2. Keep the full gz gate green while tightening compatibility facades and
   downstream migration evidence. The current DART-side regressions cover
   custom mesh-plane contacts, stacked-cylinder detachable joints, axial and
   tilted cylinder/plane-like-box support, capped large flat box/mesh contact
   patches, and FCL/ODE legacy facade raycast compatibility.
3. Finish reference-engine isolation by auditing target links, dependency
   metadata, wheel artifacts, and remaining downstream paths after the CMake
   test/benchmark opt-out, normal pixi default-off, explicit reference opt-in,
   reference target split, native-backed compatibility component facades,
   core-link, fresh runtime-link, package-export, Pixi dependency-metadata
   evidence, and repaired-head workflow-dispatch full wheel-matrix evidence.
   Local py312 wheel artifact inspection is complete and the wheel verifier
   enforces collision-runtime isolation across every wheel verify entry; keep
   final PR-state packaging evidence current.
4. Finish removing FCL, Bullet, and ODE from default package/runtime surfaces
   by preserving explicit reference test/benchmark jobs, native-backed
   compatibility facades, and wheel/package evidence across CI.
5. Continue hardening `dart/collision/` as one built-in stack after the source
   split: factory keys, direct public C++ legacy `create()` entry points,
   package component names, installed/source-tree compatibility headers, and
   user-facing examples/tutorials are native-backed; dartpy uses only the clean
   `DartCollisionDetector` API;
   old-engine implementation files are now explicit reference surfaces, with
   runtime source isolation checked by lint. Use `01-design.md` as the
   architecture checklist for remaining CI, migration, and performance gates.
6. Keep the documented downstream migration/deprecation path current for legacy
   C++ detector names and factory aliases in gz-physics and package smoke jobs.
   Dartpy does not retain legacy detector aliases through DART 7. The
   current package smoke now passes against installed native-backed
   compatibility components and headers; downstream CI/deprecation evidence is
   still required before compatibility facades can be removed.
7. Keep the scheduled/manual CI benchmark guard artifact evidence current for
   native core and public `DartCollisionDetector` scenarios, then delete legacy
   runtime backend source/components only once the final gates pass.
8. Only then package the final PR: transfer evidence from this folder into the
   PR description and remove this working folder in the same PR.

## Single-PR North-Star Epics

These epics are not separate follow-up PRs. They are the remaining phases of
the same PR toward full removal of FCL, Bullet, and ODE from DART's normal
collision stack.

1. **CI Hardening**
   - Add permanent CI coverage for native-default builds with
     `DART_BUILD_COLLISION_REFERENCE_TESTS=OFF` and
     `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF`; the initial Linux
     native-only job is now in the working tree with matching local command
     evidence and still needs GitHub CI evidence.
   - Include dartpy import smoke, the `collision-native` label, focused
     default-detector tests, and gz-physics compatibility coverage.
2. **Reference Test And Benchmark Harness**
   - Keep FCL, Bullet, and ODE available only for correctness comparisons and
     benchmark comparisons.
   - Reference comparison code now calls explicit `createReference()` APIs on
     the FCL, Bullet, and ODE detector classes, and public legacy `create()`
     paths now return native-backed compatibility facades without losing
     old-engine reference coverage.
   - The working tree adds CMake opt-out paths for reference tests and
     benchmarks, validates both reference-disabled and reference-enabled
     focused builds locally, defaults normal pixi configure paths to native-only
     collision, and uses override variables for explicit reference opt-in.
     The toggles propagate through the main debug, dartpy, install, coverage,
     ASAN, Windows, and wheel configure entry points.
   - Core native-only link, fresh runtime install, and installed
     CMake/pkg-config metadata inspection show no FCL, Bullet, ODE, libccd, or
     reference-engine runtime targets in the normal native install. Retained
     `collision-fcl`, `collision-bullet`, and `collision-ode` package
     component targets are native-backed interface facades. Default and wheel
     Pixi lock metadata no longer include old collision engines or their FCL
     transitive packages. The repaired py312 wheel artifact has also been
     inspected for old component files and runtime links, and `wheel-verify`
     now rejects those files for every matrix entry. Repaired-head
     workflow-dispatch wheel-matrix evidence passed for Ubuntu, macOS, and
     Windows on Python 3.12, 3.13, and 3.14; remaining work is keeping final
     PR-state evidence current so reference engines cannot leak into normal
     runtime targets.
3. **Backend Removal From Defaults**
   - Move FCL, Bullet, and ODE out of default packaging/runtime surfaces.
   - Keep old backends only in explicit reference/benchmark jobs while they are
     still useful for regression detection.
   - Normal pixi configure entry points now request native-only collision by
     default; old engines and comparison harnesses are enabled only by
     override.
   - Default and wheel Pixi environments no longer lock FCL, Bullet, ODE, or
     their FCL transitive packages; the explicit `collision-reference`
     environment owns those packages for comparison work.
   - A repaired py312 wheel has no old collision component headers, old
     collision component libraries, old collision CMake exports, or FCL,
     Bullet, ODE, or libccd runtime links; the wheel verifier now checks those
     forbidden artifacts for each wheel matrix entry.
   - Default `find_package(DART)` now adds only the `dart` component; it no
     longer auto-adds legacy collision components or emits deprecated
     Bullet/ODE component text from generated native-only package exports.
   - Retained `collision-fcl`, `collision-bullet`, and `collision-ode` package
     components are native-backed interface facades for downstream source
     compatibility; old-engine comparison libraries use explicit test-only
     `dart-test-reference-*` names and no package components.
   - User-facing examples/tutorials no longer require or link old collision
     component targets in their CMake. Source that previously selected Bullet
     or ODE now requests `CollisionDetectorType::Dart` or keeps the default
     built-in detector path.
4. **Collision Abstraction Cleanup**
   - Replace real legacy backend selection in `dart/collision/` with one
     built-in detector implementation.
   - The retained factory keys now resolve to `DartCollisionDetector` even when
     test-only reference targets are linked. Dartpy exposes
     `DartCollisionDetector` without retaining legacy detector aliases and no
     longer links legacy collision component targets. Direct public C++ legacy `create()`
     entry points now return native-backed facades that may preserve legacy
     display type strings for gz-physics compatibility, while C++ tests and
     benchmarks use `dart/test/reference_collision/...` includes and
     `createReference()` APIs for old-engine comparisons. Top-level source-tree
     detector/group headers are native-backed facades, old-engine
     implementation files live under `tests/dart/test/reference_collision/`,
     and lint fails if runtime DART source includes
     old-engine or reference-backend headers.
   - Preserve a clean internal architecture: public API/compatibility shell,
     DART adapter layer, native scene/query core, and optional reference
     harnesses outside runtime targets. This is an API cleanliness,
     scalability, and performance gate, not only a naming cleanup.
   - Complete the built-in component design as code, not just documentation:
     public APIs stay engine-neutral, the adapter owns scalable persistent
     scene/query synchronization, and native hot paths own compact geometry,
     broadphase, narrowphase, caches, profiler labels, and benchmark coverage.
   - Require scalable native scene state, persistent broadphase data, batched
     query paths, clear cache invalidation including dynamic-vertex shapes,
     DART filters adapted into native pair checks before narrowphase,
     deterministic results, and profiler/benchmark hooks.
5. **Downstream Migration**
   - Keep gz-physics green while using the documented migration path away from
     legacy detector names and factory aliases.
   - Do not remove compatibility facades until downstream code has a tested
     native-backed path.
   - Current gz-physics evidence is locally green: a fresh
     `pixi run -e gazebo test-gz` passed 65/65 tests against the DART plugin.
     CI evidence is still required.
6. **Performance Guardrails**
   - The current `bm-collision-check` task runs checked narrowphase, distance,
     raycast, mixed-primitive, mesh-heavy, raycast-batch, and public adapter
     benchmark subsets.
   - The latest local run recorded in `4b155655890` passed all checked subsets
     and wrote `.benchmark_results/collision_check_*.json` outputs.
   - CI Linux now has a scheduled/manual `Collision Benchmark Guard` job that
     runs the same broad guard in the `collision-reference` environment and
     uploads benchmark JSON artifacts; run `25887939088` uploaded
     `collision-benchmark-guard-25887939088-1` for repaired head
     `1e1faf6feb1`.
   - Track primitive, narrow-phase, supported distance, raycast, batch-raycast,
     mesh-heavy, mixed-primitive, and larger dirty-world simulation workloads
     through JSON artifacts.
   - Treat benchmarks as optimization evidence after correctness is locked, not
     as a substitute for correctness tests.
7. **Final Runtime Cleanup**
   - After downstream compatibility windows close, delete old external-engine
     implementation source/components from the runtime layer, retain only
     native-backed compatibility facades required by downstream migration,
     remove legacy package dependencies from default builds, and simplify CMake
     and installed-package exports around native collision. Keep the runtime
     source isolation check as the local guard that prevents old-engine runtime
     includes or implementation sources from returning.
8. **Final PR Packaging**
   - Remove this dev-task folder after transferring its evidence to the PR
     description.
   - Keep the durable built-in collision architecture notes now seeded in
     `docs/onboarding/architecture.md`.
   - Preserve final evidence for `pixi run test-all`, gz-physics, native-only
     CI, packaging inspection, and benchmark guardrails.
   - Current PR #2652 is closed per user direction. Continue preparing focused
     fixes locally when useful, publish them to `feature/new_coll` only after
     explicit user/maintainer approval, and do not create a new diff or review
     request until explicitly asked. Feature-branch pushes do not start the main
     workflows, so the latest pushed audit/docs heads have no attached runs
     unless the maintainer explicitly chooses a trigger surface.

## Detailed Planning Docs

- `01-design.md`: target single built-in architecture and API/scaling/perf
  design rules.
- `02-milestones.md`: single-PR phase gates and verification criteria.
- `03-evidence-gates.md`: command evidence and north-star progress scale.
- `04-reference-gap-analysis.md`: feature/API/performance gap matrix and
  implementation-ready architecture for the next coding phase.
- `05-downstream-migration.md`: downstream compatibility contract and removal
  gates for legacy detector names.
- `06-completion-audit.md`: completion audit that maps each north-star
  requirement to concrete evidence and remaining blockers.
- `RESUME.md`: current branch state and session handoff context.

## Completion Rules

This folder is working documentation. When native collision reaches the
completion criteria, keep only durable design notes in onboarding docs, move
final command evidence to the PR description, then delete this folder in the
same PR. Use `07-pr-evidence-transfer.md` as the starting point after the
maintainer opens the successor PR or explicitly chooses another review surface.
