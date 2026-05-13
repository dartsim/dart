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
- [x] gz-physics compatibility is proven without downstream patches.
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
- [ ] The single north-star PR is not complete yet. The checkpoint commit proves
      native default, feature parity, gz-physics compatibility, performance,
      disabled-legacy-backend builds, native-only pixi defaults, and explicit
      reference opt-in locally; the remaining work is CI hardening,
      packaging/runtime dependency removal, remaining `dart/collision/`
      class/component cleanup into one built-in detector, downstream migration
      safety, recurring performance guardrails, and final legacy backend
      deletion.

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

| Stage | Progress marker                              | Status                           |
| ----- | -------------------------------------------- | -------------------------------- |
| 0     | Baseline native backend exists               | Complete before this task        |
| 1     | Native `dart` detector is the default path   | Complete in checkpoint           |
| 2     | DART feature parity gaps are closed          | Complete in checkpoint           |
| 3     | gz-physics compatibility is proven           | Complete in checkpoint           |
| 4     | Native beats legacy backends in benchmarks   | Complete in checkpoint           |
| 5     | FCL/Bullet/ODE are optional for local builds | Complete in checkpoint           |
| 6     | Native-only and gz-physics CI are permanent  | Started; local evidence          |
| 7     | Reference engines are test/bench-only        | Started; default-off + opt-in    |
| 8     | Default packages have no old runtime deps    | Started; pixi/install proof      |
| 9     | Downstream migration/deprecation path exists | Started; DART alias coverage     |
| 10    | Collision abstraction is one clean stack     | Started; architecture documented |
| 11    | Old runtime backend source is deleted        | Not started                      |
| 12    | Final one-PR validation and PR packaging     | Blocked on stages 6-11           |

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
- FCL, Bullet, and ODE may remain only as optional reference engines for tests
  and benchmarks. Native-only builds must be able to opt out through CMake, and
  no default runtime target may depend on them.
- Performance claims require benchmark output checked into this task or linked
  from CI artifacts. Correctness and feature coverage are prerequisites; after
  that, performance work can gradually optimize native until it beats the
  reference engines on required workloads.

## Immediate Next Steps

1. Run and harden the new native-only CI job alongside existing gz-physics CI.
2. Finish reference-engine isolation by auditing target links, dependency
   metadata, wheel artifacts, and remaining downstream-component paths after
   the CMake test/benchmark opt-out, normal pixi default-off, explicit
   reference opt-in, core-link, fresh runtime-link, and package-export
   evidence.
3. Remove FCL, Bullet, and ODE from default package/runtime surfaces while
   preserving explicit reference test/benchmark jobs and native-backed
   compatibility facades.
4. Continue collapsing `dart/collision/` so retained legacy classes, headers,
   and component names are native-backed compatibility facades or explicit
   reference-only surfaces; factory keys are already native-backed aliases.
5. Define and test the downstream migration/deprecation path for legacy detector
   names and factory aliases.
6. Add recurring benchmark guardrails, then delete legacy runtime backend
   source/components once the gates pass.
7. Only then package the final PR: transfer evidence from this folder into the
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
   - The working tree adds CMake opt-out paths for reference tests and
     benchmarks, validates both reference-disabled and reference-enabled
     focused builds locally, defaults normal pixi configure paths to native-only
     collision, and uses override variables for explicit reference opt-in.
     The toggles propagate through the main debug, dartpy, install, coverage,
     ASAN, Windows, and wheel configure entry points.
   - Core native-only link, fresh runtime install, and installed
     CMake/pkg-config metadata inspection show no FCL, Bullet, ODE, libccd, or
     old collision component targets in the normal native install. Remaining
     work is broader dependency-metadata, wheel-artifact, and
     downstream-component inspection so reference engines cannot leak into
     normal runtime targets.
3. **Backend Removal From Defaults**
   - Move FCL, Bullet, and ODE out of default packaging/runtime surfaces.
   - Keep old backends only in explicit reference/benchmark jobs while they are
     still useful for regression detection.
   - Normal pixi configure entry points now request native-only collision by
     default; old engines and comparison harnesses are enabled only by
     override.
   - Default `find_package(DART)` now adds only the `dart` component; it no
     longer auto-adds legacy collision components or emits deprecated
     Bullet/ODE component text from generated native-only package exports.
4. **Collision Abstraction Cleanup**
   - Replace real legacy backend selection in `dart/collision/` with one
     built-in detector implementation.
   - The retained factory keys now resolve to `DartCollisionDetector` even when
     legacy reference components are linked. Remaining work is direct legacy
     detector class/header/component cleanup so those surfaces are wrappers or
     explicit reference-only APIs.
   - Preserve a clean internal architecture: public API/compatibility shell,
     DART adapter layer, native scene/query core, and optional reference
     harnesses outside runtime targets. This is an API cleanliness,
     scalability, and performance gate, not only a naming cleanup.
   - Require scalable native scene state, persistent broadphase data, batched
     query paths, clear cache invalidation including dynamic-vertex shapes,
     DART filters adapted into native pair checks before narrowphase,
     deterministic results, and profiler/benchmark hooks.
5. **Downstream Migration**
   - Keep gz-physics green while documenting the migration path away from
     legacy detector names and factory aliases.
   - Do not remove compatibility facades until downstream code has a tested
     native-backed path.
6. **Performance Guardrails**
   - Promote the current comparative benchmark evidence into recurring
     regression checks.
   - Track primitive, narrow-phase, supported distance, raycast, batch-raycast,
     mesh-heavy, mixed-primitive, and larger dirty-world simulation workloads.
   - Treat benchmarks as optimization evidence after correctness is locked, not
     as a substitute for correctness tests.
7. **Final Deletion**
   - After downstream compatibility windows close, delete old backend
     source/components from the runtime backend layer, remove legacy package
     dependencies from default builds, and simplify CMake and installed-package
     exports around native collision.
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
- `RESUME.md`: current branch state and session handoff context.

## Completion Rules

This folder is working documentation. When native collision reaches the
completion criteria, extract only durable design notes into
`docs/onboarding/architecture.md`, then delete this folder in the same PR.
