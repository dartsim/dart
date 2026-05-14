# Native Collision North-Star PR - Dev Task

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
- [ ] gz-physics compatibility is proven without downstream patches for the
      current source state. The latest focused work fixed the DART/gz custom
      mesh-plane free-fall by enforcing collision-object-order contact normals
      through native dispatch, and fixed stacked cylinder support by adding
      parallel-cylinder cap/side contact selection. The latest checkpoint also
      adds axial cylinder-cap support patches against large boxes, matching the
      gz plane-as-box contact path. Focused gz runs now pass
      `COMMON_TEST_collisions_dartsim`,
      `COMMON_TEST_detachable_joint_dartsim`, and
      `COMMON_TEST_joint_transmitted_wrench_features_dartsim`. The gate remains
      open because `COMMON_TEST_joint_features_dartsim` still fails the
      `JointDetach` exact-zero off-axis velocity checks after richer
      box/cylinder contact support.
- [x] Comparative benchmarks prove native is at least as fast as the best
      legacy backend for required workloads. Primitive, narrow-phase,
      supported distance, raycast, raycast-batch, mesh-heavy, and
      mixed-primitive scenario cases pass. The previous mixed-primitives dense
      1000 loss is now a win through cached broad-phase snapshot reuse:
      native is 1.06 ms CPU mean versus Bullet at 1.69 ms.
- [x] FCL, Bullet, and ODE are no longer required collision dependencies. A
      core `dart` build, focused native/default C++ tests, and `dartpy` now
      build with all three disabled.
- [x] Normal pixi configure paths now default FCL, Bullet, ODE, reference
      correctness tests, and reference benchmarks to `OFF`; comparison jobs
      opt in with `DART_BUILD_COLLISION_*_OVERRIDE=ON`.
- [x] Native-only install metadata no longer advertises old collision
      component targets or old collision runtime libraries; the remaining
      installed collision option variables are `OFF` state reporting.
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
      FCL/Bullet/ODE collision component exports, reference collision
      libraries, and FCL, Bullet, ODE, or libccd runtime libraries from dartpy
      wheel artifacts. The existing repaired py312 Linux wheel passes that
      check locally.
- [x] `dartpy` no longer links legacy collision component targets. The Python
      compatibility names `DARTCollisionDetector`, `FCLCollisionDetector`,
      `BulletCollisionDetector`, and `OdeCollisionDetector` now construct and
      report the built-in `dart` detector even in a reference-enabled build
      where FCL, Bullet, and ODE components exist.
- [x] The `collision-reference` environment configures with FCL, Bullet, ODE,
      reference tests, and reference benchmarks enabled, and the focused
      `test_reference_backends` target builds and passes.
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
- [x] Optional old-engine CMake targets and components are now explicitly
      reference-named: `dart-collision-reference-fcl`,
      `dart-collision-reference-bullet`, `dart-collision-reference-ode`, and
      matching `collision-reference-*` package components.
- [x] Retained legacy package component names `collision-fcl`,
      `collision-bullet`, and `collision-ode` are native-backed interface
      facades. A default native-only install/export probe and downstream
      package smoke test that requests `collision-fcl`, `collision-bullet`, and
      `collision-ode` proves those names link the built-in `dart` stack and do
      not install old collision libraries.
- [x] Installed legacy detector headers for FCL, Bullet, and ODE are now
      compatibility facades over `DartCollisionDetector` in native-only and
      reference-enabled installs. The downstream package/header smokes include
      the installed legacy detector headers and verify factory keys plus legacy
      class `create()` calls all report detector type `dart`.
- [x] `05-downstream-migration.md` defines the downstream compatibility
      contract: retained legacy detector names, factory keys, Python names, and
      package components are source-compatible aliases for the built-in
      detector, while reference engines require explicit `collision-reference-*`
      targets and `createReference()` APIs.
- [x] Source-tree legacy detector headers are now split the same way as the
      installed package surface: top-level FCL, Bullet, and ODE detector/group
      headers, PascalCase compatibility headers, All headers, and component
      headers are native-backed compatibility facades, while old-engine
      implementation headers and sources live under explicit `reference/`
      paths used by reference tests and benchmarks.
- [x] A runtime source isolation check is wired into `lint` and `check-lint`.
      It fails if non-reference DART source paths include FCL, Bullet, ODE,
      libccd, or explicit collision reference backend headers, or if legacy
      engine implementation sources move back outside `reference/` paths.
- [x] A broad recurring benchmark guard exists:
      `pixi run -e collision-reference bm-collision-check` builds and runs
      checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy,
      raycast-batch, and public DART adapter benchmark subsets. Native/reference
      subsets emit Google Benchmark JSON and compare native timings against the
      best enabled FCL/Bullet/ODE reference result; the public adapter subset
      records JSON for the built-in `DartCollisionDetector` path.
- [x] CI Linux has a scheduled/manual collision benchmark guard job that runs
      the broad `collision-reference` benchmark check and uploads the
      `.benchmark_results/collision_check_*.json` artifacts.
- [ ] The single north-star PR is not complete yet. The checkpoint commit proves
      native default, feature parity, gz-physics compatibility, performance,
      disabled-legacy-backend builds, native-only pixi defaults, and explicit
      reference opt-in locally. The current dependency-metadata slice also
      proves default/wheel Pixi metadata isolation and a repaired py312 wheel
      artifact inspection. The current API-cleanup slice also proves
      native-backed Python compatibility aliases and reference-enabled dartpy
      link isolation, explicit C++ reference-detector creation for
      tests/benchmarks, and native-backed direct C++ legacy `create()` entry
      points. User-facing examples/tutorials have also been moved off old
      collision components, and package component names are now split into
      native-backed compatibility facades versus explicit reference targets.
      Installed and source-tree legacy detector headers are also native-backed
      facades, with old-engine implementation files under explicit reference
      paths, and lint now guards that runtime source isolation. The remaining
      work is CI hardening, full wheel matrix/CI artifact evidence from the
      wired verifier, downstream migration run evidence, GitHub evidence for
      the scheduled performance guard, explicit API/scalability/performance
      architecture gate evidence, and final legacy backend deletion.

## Goal

Make DART's built-in/native collision detector the default implementation and
the long-term replacement for FCL, Bullet, and ODE collision backends. The
replacement must be evidence driven: feature coverage, behavior compatibility,
gz-physics compatibility, and performance must be verified before dependency
removal.

## North Star

DART should not require FCL, Bullet, or ODE for collision detection. The native
detector should cover the DART collision feature surface and beat the best
legacy backend on representative collision, distance, raycast, broadphase, and
simulation workloads. `dart/collision/` should expose one built-in collision
detector stack, not a real multi-backend selection layer. Any legacy detector
name, header, factory key, or component retained for gz-physics compatibility
must be a thin adapter over the built-in detector.

The built-in detector must also have a clean component architecture: public
DART collision APIs and compatibility facades at the outside, a DART adapter
layer for shape/result/filter conversion, and a native scene/query core for
geometry, broadphase, narrowphase, distance, raycast, contact persistence,
caching, and profiling. Public APIs should expose DART semantics rather than
engine-specific knobs, while the native core stays scalable and
performance-oriented internally.

FCL, Bullet, and ODE may remain only as optional reference engines for
correctness tests and benchmarks. They should be controlled by CMake options so
comparison jobs can opt in and native-only builds can opt out. The work order
inside this PR is feature coverage first, correctness tests as the permanent
gate, then gradual performance optimization until native beats the reference
engines on required workloads.

## North-Star Progress Scale

This task is scoped as one PR that continues until the north star is reached.
The current checkpoint is a validated middle state, not a final PR boundary.

| Stage | Progress marker                              | Status                             |
| ----- | -------------------------------------------- | ---------------------------------- |
| 0     | Baseline native backend exists               | Complete before this task          |
| 1     | Native `dart` detector is the default path   | Complete in checkpoint             |
| 2     | DART feature parity gaps are closed          | Complete in checkpoint             |
| 3     | gz-physics compatibility is proven           | Mostly repaired; one focused fail  |
| 4     | Native beats legacy backends in benchmarks   | Complete in checkpoint             |
| 5     | FCL/Bullet/ODE are optional for local builds | Complete in checkpoint             |
| 6     | Native-only and gz-physics CI are permanent  | Started; gz focused gate not final |
| 7     | Reference engines are test/bench-only        | Local target split proven          |
| 8     | Default packages have no old runtime deps    | Local pass; CI verifier wired      |
| 9     | Downstream migration/deprecation path exists | Plan documented; run evidence left |
| 10    | Collision abstraction is one clean stack     | Facades proven; arch gates open    |
| 11    | Old runtime backend source is reference-only | Local split; lint guard wired      |
| 12    | Final one-PR validation and PR packaging     | Blocked on CI/migration/arch/del   |

## Built-In Architecture Status

`01-design.md` is the architecture and design contract for the built-in
collision component. The target layer shape is public DART collision APIs and
temporary compatibility facades at the outside, `dart/collision/dart/` as the
DART shape/result/filter adapter, and `dart/collision/native/` as the scalable
scene/query core for geometry, broadphase, narrowphase, distance, raycast,
contact/manifold caching, deterministic results, profiling, and benchmarks.
Its "Built-In Component Blueprint" is the concrete design checklist for API
cleanliness, adapter-owned scene state, native query lifecycle, scalability,
and performance-oriented internals.

The implementation now satisfies the source/package structure of that design:
public factory keys, Python compatibility names, retained package components,
installed legacy headers, source-tree legacy detector/group/All/PascalCase
headers, and direct legacy `create()` entry points route to the built-in `dart`
detector. Old FCL/Bullet/ODE implementation headers and sources live under
explicit `reference/` paths and build only through reference test/benchmark
targets. The `check-collision-runtime-isolation` task now guards the runtime
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
matching semantics. The architecture gate remains open until alias cleanup,
package cleanup, and native algorithms are backed by design evidence in three
dimensions:

- API cleanliness: ordinary users see one `dart` detector surface, semantic
  DART query options/results, native-backed compatibility names, and explicit
  reference-only APIs for old engines.
- Scalability: public collision, distance, and raycast queries use
  adapter-owned persistent scene state, stable native IDs, dirty sync,
  deterministic result assembly, and deliberate cache invalidation.
- Performance orientation: hot paths use compact native geometry,
  shape-specialized dispatch, persistent broadphase/query state, reusable
  scratch/cache lifetimes, and benchmark/profiler labels for each query stage.

That gate remains open for CI evidence, downstream migration evidence, the
remaining gz-physics `JointDetach` residual, and broader
correctness/performance guardrails across the public DART adapter and native
core paths.

## Design Readiness Tracker

The status below is measured against the north star, not against whether a
single checkpoint built locally.

| Design axis             | North-star bar                                                                                                                                                                                                                                                    | Current state                                                                                                                                                                                                                                                                                                                                                                     |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Component layering      | Public DART APIs and compatibility facades sit outside `dart/collision/dart/`; the DART adapter owns scene synchronization and result conversion; `dart/collision/native/` owns algorithms, broadphase, query state, caches, and profiling.                       | The source/package split now matches this shape: legacy public paths are native-backed facades, reference implementation files are under explicit `reference/` paths, and lint guards runtime source isolation. CI/downstream and final deletion evidence remain before the layer is final.                                                                                       |
| API cleanliness         | `dart` is the canonical public detector; legacy keys, classes, headers, and package components are compatibility facades only; public options/results describe DART semantics instead of backend-specific modes.                                                  | Factory aliases, Python names, public C++ legacy `create()` paths, installed headers, source-tree top-level legacy headers, examples, and retained package components are native-backed. CI/downstream migration evidence is still needed before this is final.                                                                                                                   |
| Scalability             | Public collision, distance, and raycast use persistent adapter scene state with stable IDs, dirty transform/shape sync, reusable broadphase/query data, cache invalidation, deterministic ordering, and contact results that are stable under pair-order changes. | Persistent `DartCollisionGroup` scene state, broadphase-pruned raycast, AABB-pruned distance, native filter adaptation, dynamic-shape invalidation coverage, scene-issued manifold cache IDs, and pair-order normal tests are implemented locally. Broader CI and recurring benchmark evidence remain.                                                                            |
| Performance orientation | Native hot paths use compact geometry, shape-specialized dispatch, persistent broadphase data, reusable scratch, clear cache lifetimes, and profiling/benchmark labels for each query stage.                                                                      | Recorded benchmarks show native wins on the measured primitive, narrowphase, supported distance, raycast, batch, mesh-heavy, and mixed-primitive set. The native dispatcher keeps canonical shape-specialized functions while wrapping only result-normal orientation when needed. The recurring benchmark guard covers checked native-vs-reference and public adapter scenarios. |
| Reference isolation     | FCL, Bullet, and ODE exist only as optional reference engines for tests and benchmarks, with native-only builds able to opt out.                                                                                                                                  | CMake opt-out options, native-only Pixi defaults, explicit `collision-reference` opt-in, `collision-reference-*` targets, reference-path source split, runtime source isolation linting, package/wheel metadata cleanup, local install/wheel evidence, and wheel artifact verifier wiring are in place. CI wheel-matrix run evidence remains.                                     |
| Compatibility           | gz-physics and downstream source-compatible legacy names keep building during migration, but cannot select an external runtime engine.                                                                                                                            | Legacy detector headers/classes, factory aliases, Python names, retained package components, and the documented migration contract route to native. Direct legacy facade display strings are covered by DART tests. Focused gz collision, detachable-joint, and transmitted-wrench tests pass; `JointDetach` exact-zero velocity remains open before the gate can close.          |

## Architecture Completion Rubric

Stage 10 on the north-star scale is complete only when the built-in collision
component is clean as an API, scalable as a scene/query system, and ready for
performance work without reopening public backend selection.

| Design gate             | Completion bar                                                                                                                                     | Current evidence                                                                                                                 | Still needed                                                                                                |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| API cleanliness         | Public DART APIs expose one canonical `dart` detector; legacy names are native-backed facades; reference engines use explicit test/benchmark APIs. | Factory aliases, Python names, C++ legacy `create()` paths, installed/source headers, and package facades route native.          | CI/package matrix evidence and final search proving no public path can select an external runtime backend.  |
| Scalable adapter/core   | `DartCollisionGroup` owns persistent scene state, stable IDs, dirty sync, cache invalidation, deterministic results, and reusable query snapshots. | Public adapter tests and benchmarks cover dirty sync, dynamic geometry invalidation, filters, pair-order normals, and cache IDs. | Finish gz `JointDetach` support-contact residual and broaden recurring public-adapter correctness evidence. |
| Performance orientation | Native hot paths use compact geometry, shape-specialized dispatch, persistent broadphase data, reusable scratch/caches, and measured query stages. | Native-vs-reference benchmarks pass locally across primitive, distance, raycast, batch, mesh-heavy, and mixed workloads.         | CI benchmark artifact evidence and any optimization needed after correctness gates are permanently green.   |
| Reference isolation     | FCL, Bullet, and ODE are optional reference engines only; native-only builds, installs, wheels, and downstream facades do not link them.           | CMake opt-out, reference targets, lint source isolation, install/package/wheel checks, and wheel verifier wiring exist.          | CI wheel matrix and downstream package smoke evidence.                                                      |
| Compatibility facade    | gz-physics-required spellings compile and run through the built-in detector while migration removes reliance on legacy names.                      | DART facade tests and focused gz collision, detachable-joint, and transmitted-wrench tests pass.                                 | Full gz gate after the `JointDetach` residual is fixed or accepted through a documented downstream path.    |

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
- Reference backends are validation tools during migration, not permanent
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

1. Finish the remaining gz-physics compatibility regression: the custom
   mesh-plane free-fall, stacked-cylinder detachable-joint, and axial
   cylinder-cap/large-box support paths now have DART-side regressions and
   focused gz evidence. Reduce the remaining
   `COMMON_TEST_joint_features_dartsim` `JointDetach` exact-zero velocity
   residual, then rerun the full gz gate without selecting FCL, Bullet, or ODE.
   The latest temporary diagnostic run shows the upper-link off-axis residual
   follows base support motion: by step 9 the base angular velocity around
   `Y` is `-6.6445e-05`, the upper-link angular `Y` is `-6.6438e-05`, and the
   upper-link linear `X` residual is consistent with that base rotation through
   the joint offset. Continue the reduction as a base-vs-ground support
   contact/manifold issue, not as a detach-state restoration issue.
   Do not retry broad cylinder-cap/large-box support-contact shaping without a
   new reduction: tilted, mirrored, centered, and weighted-centroid variants all
   passed or could be made to pass focused native tests but either killed the
   expected upward `Z` motion in gz `JointDetach` or worsened off-axis support
   motion.
2. Run and harden the new native-only CI job alongside existing gz-physics CI.
3. Finish reference-engine isolation by auditing target links, dependency
   metadata, wheel artifacts, and remaining downstream paths after the CMake
   test/benchmark opt-out, normal pixi default-off, explicit reference opt-in,
   reference target split, native-backed compatibility component facades,
   core-link, fresh runtime-link, package-export, and Pixi dependency-metadata
   evidence. Local py312 wheel artifact inspection is now complete and the
   wheel verifier enforces collision-runtime isolation; CI wheel matrix
   artifacts still need run evidence.
4. Finish removing FCL, Bullet, and ODE from default package/runtime surfaces
   by preserving explicit reference test/benchmark jobs, native-backed
   compatibility facades, and wheel/package evidence across CI.
5. Continue hardening `dart/collision/` as one built-in stack after the source
   split: factory keys, Python detector names, direct public C++ legacy
   `create()` entry points, package component names, installed/source-tree
   compatibility headers, and user-facing examples/tutorials are native-backed;
   old-engine implementation files are now explicit reference surfaces, with
   runtime source isolation checked by lint. Use `01-design.md` as the
   architecture checklist for remaining CI, migration, and performance gates.
6. Test the documented downstream migration/deprecation path for legacy
   detector names and factory aliases in gz-physics and package smoke jobs.
7. Run the scheduled/manual CI benchmark guard and collect artifact evidence for
   native core and public `DartCollisionDetector` scenarios, then delete legacy
   runtime backend source/components once the gates pass.
8. Only then package the final PR: transfer evidence from this folder into the
   PR description and remove this working folder in the same PR.

## Single-PR North-Star Epics

These epics are not separate follow-up PRs. They are the remaining phases of
the same PR toward full removal of FCL, Bullet, and ODE from DART's normal
collision stack.

1. **CI Hardening**
   - Add permanent CI coverage for native-default builds with
     `DART_BUILD_COLLISION_FCL=OFF`, `DART_BUILD_COLLISION_BULLET=OFF`, and
     `DART_BUILD_COLLISION_ODE=OFF`; the initial Linux native-only job is now
     in the working tree with matching local command evidence and still needs
     GitHub CI evidence.
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
     now rejects those files for every matrix entry. Remaining work is CI
     wheel-matrix run/artifact evidence so reference engines cannot leak into
     normal runtime targets.
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
     compatibility; old-engine comparison libraries/components use explicit
     `collision-reference-*` names.
   - User-facing examples/tutorials no longer require or link old collision
     component targets in their CMake. Source that previously selected Bullet
     or ODE now requests `CollisionDetectorType::Dart` or keeps the default
     built-in detector path.
4. **Collision Abstraction Cleanup**
   - Replace real legacy backend selection in `dart/collision/` with one
     built-in detector implementation.
   - The retained factory keys now resolve to `DartCollisionDetector` even when
     legacy reference components are linked. Python detector compatibility
     names also resolve to `DartCollisionDetector` and dartpy no longer links
     legacy collision component targets. Direct public C++ legacy `create()`
     entry points now return native-backed facades that may preserve legacy
     display type strings for gz-physics compatibility, while C++ tests and
     benchmarks use explicit `reference/` includes and `createReference()` APIs
     for old-engine comparisons. Top-level source-tree detector/group headers
     are native-backed facades, old-engine implementation files live under
     reference-only paths, and lint fails if runtime DART source includes
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
   - Current focused gz-physics evidence is partially green: direct legacy
     display strings, `MeshAndPlane`, detachable joints, and transmitted wrench
     features are fixed, but `JointDetach` exact-zero velocity remains open.
6. **Performance Guardrails**
   - The current `bm-collision-check` task runs checked narrowphase, distance,
     raycast, mixed-primitive, mesh-heavy, raycast-batch, and public adapter
     benchmark subsets.
   - CI Linux now has a scheduled/manual `Collision Benchmark Guard` job that
     runs the same broad guard in the `collision-reference` environment and
     uploads benchmark JSON artifacts.
   - Track primitive, narrow-phase, supported distance, raycast, batch-raycast,
     mesh-heavy, mixed-primitive, and larger dirty-world simulation workloads
     through JSON artifacts.
   - Treat benchmarks as optimization evidence after correctness is locked, not
     as a substitute for correctness tests.
7. **Final Deletion**
   - After downstream compatibility windows close, delete old backend
     source/components from the runtime backend layer, remove legacy package
     dependencies from default builds, and simplify CMake and installed-package
     exports around native collision. Keep the runtime source isolation check
     as the local guard that prevents old-engine runtime includes or
     implementation sources from returning.
8. **Final PR Packaging**
   - Remove this dev-task folder after transferring its evidence to the PR
     description.
   - Keep durable architecture notes in `docs/onboarding/architecture.md`.
   - Preserve final evidence for `pixi run test-all`, gz-physics, native-only
     CI, packaging inspection, and benchmark guardrails.

## Detailed Planning Docs

- `01-design.md`: target single built-in architecture and API/scaling/perf
  design rules.
- `02-milestones.md`: single-PR phase gates and verification criteria.
- `03-evidence-gates.md`: command evidence and north-star progress scale.
- `04-reference-gap-analysis.md`: feature/API/performance gap matrix and
  implementation-ready architecture for the next coding phase.
- `05-downstream-migration.md`: downstream compatibility contract and removal
  gates for legacy detector names.
- `RESUME.md`: current branch state and session handoff context.

## Completion Rules

This folder is working documentation. When native collision reaches the
completion criteria, extract only durable design notes into
`docs/onboarding/architecture.md`, then delete this folder in the same PR.
