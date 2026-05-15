# Native Collision Milestones

## PR Scope

This milestone plan is for a single north-star PR. Phase 1 through Phase 6 are
the validated native-default checkpoint. Phase 7 onward are still required in
the same PR before the PR can be considered complete. The final PR state has
one built-in collision detector implementation behind `dart/collision/`; any
legacy names that remain for compatibility are native-backed wrappers/adapters,
not selectable external backends.

The quality order is intentional: implement feature coverage first, lock
correctness through tests, and then use benchmarks to gradually optimize native
until it beats the reference engines on required workloads. FCL, Bullet, and
ODE may remain only as optional reference engines for tests and benchmarks, with
CMake options that allow native-only builds to opt out.

The architecture order is also intentional: the final API should be clean, the
built-in component should scale, and performance hooks should be part of the
native design. Public DART APIs and compatibility facades sit outside a DART
adapter layer, while `dart/collision/native/` owns scene state, broadphase,
narrowphase, query algorithms, cache lifetimes, and profiling.
`01-design.md` is the canonical architecture contract for those boundaries and
must be satisfied before the collision abstraction cleanup phase is complete.

`04-reference-gap-analysis.md` tracks the detailed feature/API/performance gaps
that make the next implementation step concrete.

## Phase 1: Native Default Path

Success criteria:

- `ConstraintSolver` default detector selection prefers `dart`.
- `experimental` remains only as a compatibility alias.
- Native contact cache and warm-start paths use the native detector after the
  rename.
- Public docs describe `dart/collision/native/` and `dart` as the default.

Verification:

- Focused unit tests for detector factory/default creation.
- Focused constraint/contact tests that exercise cached contact impulse write
  back.
- `pixi run lint`.

## Phase 2: Feature Parity Gate

Success criteria:

- Collision, distance, and raycast tests cover the shapes and filters DART uses
  from FCL, Bullet, and ODE.
- Unsupported shapes have an explicit implementation or a documented DART-level
  non-requirement.
- Native-vs-reference consistency tests cover contact sign, normal direction,
  depth, distance, nearest point, and raycast semantics.

Verification:

- Native collision unit tests.
- Collision integration tests.
- Backend consistency tests with reference backends enabled.

## Phase 3: gz-physics Gate

Success criteria:

- gz-physics builds against this branch without patches.
- gz-physics tests pass with the native detector as DART's default.
- Legacy class-name and factory-key compatibility is verified.

Verification:

- `pixi run -e gazebo test-gz`.
- Any failing gz-physics case is reduced to a DART test before being fixed.

## Phase 4: Performance Gate

Success criteria:

- Comparative benchmark results are recorded in `03-evidence-gates.md`.
- Native beats the best legacy backend in required narrowphase, broadphase,
  distance, raycast, mixed primitive, and mesh-heavy scenarios.
- No benchmark result is accepted without correctness checks for the same case.

Verification:

- Comparative benchmark suite with FCL, Bullet, and ODE enabled.
- Native-only microbenchmarks for regression tracking.
- Benchmark command lines, commit SHAs, and summary tables recorded.

## Phase 5: Dependency Removal Gate

Success criteria:

- Default DART configure/build no longer requires FCL, Bullet, or ODE for
  collision detection.
- Optional reference backend builds remain available only where needed for
  validation.
- Packaging metadata and CMake component dependencies no longer advertise old
  collision libraries as runtime requirements.

Verification:

- Configure/build with FCL, Bullet, and ODE disabled or absent.
- Install-tree inspection for unwanted runtime links.
- Public headers still satisfy gz-physics compatibility requirements.

## Phase 6: Checkpoint Evidence And PR Continuation

Success criteria:

- Stale `experimental` wording is gone except for compatibility aliases and
  simulation-experimental code.
- Dev-task learnings are condensed into onboarding docs.
- The checkpoint evidence is complete enough to continue toward the remaining
  north-star phases in the same PR.

Verification:

- `pixi run lint`.
- `pixi run test-all` when feasible.
- `pixi run -e gazebo test-gz`.
- `03-evidence-gates.md` records checkpoint evidence and remaining gates.

## Phase 7: CI Hardening

Purpose: make native-default and no-legacy-backend configurations visible in
continuous integration so future changes cannot silently reintroduce
FCL/Bullet/ODE runtime requirements.

Success criteria:

- CI has a native-default job with FCL, Bullet, and ODE disabled. The initial
  Linux job is in the working tree and awaits CI evidence.
- CI builds `dart`, `dartpy`, and native collision tests in that configuration.
- CI runs the `collision-native` label, focused default-detector C++ tests, and
  a dartpy import/simulation smoke.
- gz-physics compatibility remains covered by a job that builds all optional
  legacy components needed by downstream package exports.
- Separate reference jobs can enable FCL, Bullet, and ODE only for correctness
  comparisons and benchmarks.

Verification:

- CI job definitions explicitly keep `DART_BUILD_COLLISION_REFERENCE_TESTS=OFF`
  and `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF`.
- The disabled-backend job passes on Linux before old collision dependencies
  are removed from any wider build environment.
- gz-physics CI passes against the same branch without local downstream
  patches.
- Reference correctness and benchmark jobs prove their old-engine dependencies
  are scoped to test/benchmark targets.

## Phase 8: Reference Test And Benchmark Harness

Purpose: keep the high correctness and performance bar after runtime backends
are removed. FCL, Bullet, and ODE are reference engines only; they are not
selectable collision backends.

Current working-tree status: this phase is started. The branch now has
`DART_BUILD_COLLISION_REFERENCE_TESTS` and
`DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` CMake options, gates focused
reference-only tests and comparative benchmarks behind them, and has local
reference-disabled/reference-enabled evidence in `03-evidence-gates.md`.
Normal pixi configure paths now default reference tests and reference
benchmarks to `OFF`; reference comparison jobs opt in through
`DART_BUILD_COLLISION_REFERENCE_TESTS_OVERRIDE=ON` and
`DART_BUILD_COLLISION_REFERENCE_BENCHMARKS_OVERRIDE=ON`. There are no public
per-engine FCL/Bullet/ODE collision build switches in the current build
surface. The reference toggles now propagate through the main debug, dartpy,
install, coverage, ASAN, Windows, and wheel configure entry points. Core
native-only link and installed package-export
inspection now show no old collision component targets or old collision runtime
libraries in the native-only install metadata. A fresh native-only runtime
install probe now also covers the installed shared libraries and dartpy module
for old-engine runtime links. Default and wheel Pixi lock metadata now exclude
FCL, Bullet, ODE, and the FCL transitive packages; the explicit
`collision-reference` environment owns those packages and restores the focused
reference target. A repaired py312 wheel artifact built without normal
old-engine collision switches now imports successfully and has no old collision
component files or runtime links. Downstream-component inspection and CI
wheel-matrix evidence are still required before this phase is complete. C++
reference tests
and benchmarks now call explicit `createReference()` detector APIs, so
old-engine comparison intent is visible at each call site.
The runtime source isolation check is wired into lint so non-reference DART
source paths cannot include old-engine or reference-backend headers.

Success criteria:

- Correctness tests compare native against reference engines where the reference
  behavior is useful and well-defined.
- Benchmarks compare native against Bullet, FCL, and ODE while native is being
  optimized.
- Reference engines are isolated to test and benchmark targets.
- Reference harnesses consume public DART collision APIs or explicit test-only
  adapters; they do not become dependencies of the runtime native core.
- CMake exposes option paths that allow reference-engine jobs to opt in and
  native-only jobs to opt out.
- Native-only correctness tests and native-only benchmark baselines still run
  when all reference engines are disabled.

Verification:

- Build with reference engines enabled runs reference consistency tests and
  comparative benchmarks.
- Build with reference engines disabled still passes native correctness tests,
  native benchmarks, dartpy smoke, and gz-physics compatibility.
- Link inspection confirms normal runtime targets do not depend on FCL, Bullet,
  or ODE.

## Phase 9: Backend Removal From Default Packaging

Purpose: make native collision the only normal runtime collision stack while
retaining legacy backends only where they are intentionally requested for
reference or benchmark work.

Current working-tree status: this phase is started, not complete. Normal pixi
configure paths now request native-only collision by default, wheel CMake
defaults keep reference harnesses disabled, and a native-only install-style
build/install produced no old collision libraries or old collision component
target files. The latest package-export cleanup also removes the default
`collision-fcl` fallback and the generated
`collision-bullet`/`collision-ode` compatibility text from native-only
installed CMake metadata; the install probe now reports only
`DART_BUILD_COLLISION_REFERENCE_TESTS` and
`DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` as `OFF`. A fresh install of the
normal runtime components has no FCL, Bullet, ODE, or libccd runtime links
from installed shared libraries or the built dartpy module. Default and wheel
Pixi dependency metadata now also omit FCL, Bullet, ODE, and the FCL
transitive packages, while `collision-reference` is the explicit opt-in
package environment for reference comparisons. A repaired py312 wheel artifact
has no
old collision component headers, old collision component libraries, old
collision CMake exports, or FCL, Bullet, ODE, or libccd runtime links.
Remaining work includes CI wheel-matrix evidence and gz-physics compatibility
for any retained legacy component facades.

Success criteria:

- Default packages do not depend on FCL, Bullet, or ODE for collision
  detection.
- CMake package exports do not add optional legacy components unless those
  components were built and installed.
- Examples, Python wheels, and normal source builds work without legacy
  collision libraries installed.
- Reference engines remain available only through explicit test/benchmark
  options for validation and performance work.

Verification:

- Fresh configure/build/test from an environment without FCL, Bullet, and ODE.
- Installed package inspection shows no unwanted default runtime links to old
  collision libraries.
- Wheel/source-build jobs pass with native collision only.

## Phase 10: Downstream Migration

Purpose: remove downstream reliance on legacy detector names and factory keys
without breaking gz-physics or other packages that include DART compatibility
headers.

Current working-tree status: this phase is locally complete and needs CI plus
downstream migration/deprecation evidence. DART-side factory and SKEL
parser coverage now proves `"fcl"`, `"fcl_mesh"`, `"bullet"`, and `"ode"`
selection routes to `DartCollisionDetector`, including a reference-enabled
build where legacy component libraries are linked. Dartpy exposes
`DartCollisionDetector` without legacy detector aliases and no longer links
legacy collision component targets even in a reference-enabled build. Direct
public C++ legacy facades keep legacy display strings for
gz-physics compatibility while remaining native-backed. FCL/ODE facades now
preserve gz-required unsupported raycast behavior, while Bullet/native raycast
support remains available. A fresh local `pixi run -e gazebo test-gz` passes
65/65 tests after DART-side mesh-plane pair-order normal,
stacked-cylinder contact, axial cylinder-cap/large-box support, tilted
cylinder/plane-like-box support, and capped large flat box/mesh contact fixes.
A current native compatibility package smoke also proves retained
`collision-fcl`, `collision-bullet`, and `collision-ode` package components and
installed legacy detector headers link/use the built-in stack without
installing old-engine runtime libraries. CI evidence is still required before
this phase can complete on the release gate.

Success criteria:

- gz-physics has a documented path to lowercase native APIs and the `"dart"`
  factory key.
- Legacy uppercase detector facades remain source-compatible during the
  migration window.
- Any behavior that intentionally differs between legacy facades and native
  lowercase APIs has DART-side regression coverage.
- A deprecation policy exists for `"experimental"` and legacy backend factory
  aliases.

Verification:

- `pixi run -e gazebo test-gz` passes at each migration step.
- Downstream failures are reduced to DART tests before fixes land.
- Documentation states which compatibility names are temporary and which names
  are stable.

## Phase 11: Collision Abstraction And Built-In Architecture Cleanup

Purpose: remove the real multi-backend abstraction from `dart/collision/` so
the completed PR has one built-in collision implementation. Legacy names may
remain only as source-compatible wrappers or adapters for gz-physics and other
downstream code. This phase also locks the built-in component architecture:
API-clean public surfaces, a focused DART adapter layer, scalable native
scene/query state, and performance-oriented internals. It is not complete if
the PR only aliases old names to `dart`; the architecture must also support
scalable native queries and performance work without reopening public backend
selection.

Current working-tree status: this phase is started. Factory-level backend
selection is now native-backed: `dart`, `experimental`, `fcl`, `fcl_mesh`,
`bullet`, and `ode` all create `DartCollisionDetector` through the public
factory. Default package exports now also point users at the `dart` component
without auto-adding old collision components. The target built-in architecture
is documented in `01-design.md` as public API and compatibility facades outside a
`dart/collision/dart/` adapter, with `dart/collision/native/` owning scalable
scene/query state and performance instrumentation. The same design contract now
contains a concrete blueprint for API cleanliness, adapter-owned scene state,
native scene/query data, query lifecycle, scalability rules, and
performance-oriented internals. Dartpy exposes `DartCollisionDetector` without
legacy detector aliases and no longer links legacy collision component targets.
Reference tests and
benchmarks now use explicit `createReference()` APIs for old-engine
comparisons. Direct public C++ legacy detector `create()` paths now resolve to
native-backed `DartCollisionDetector` facades; any legacy display type string
is compatibility metadata, not backend selection. Top-level source-tree FCL,
Bullet, and ODE detector/group headers are native-backed facades, while real
FCL/Bullet/ODE implementation headers and sources live under explicit
`reference/` paths for reference-only tests and benchmarks. DART-side public
facade tests cover the direct C++ display-name compatibility needed by
gz-physics, and new pair-order normal tests cover direct native dispatch,
optimized collision-world dispatch, and the public DART collision group path.
Native tests also cover axial cylinder-cap support patches against large boxes,
matching gz's current plane-as-box fallback path, and a reduced gz-like
`JointDetach` support fixture now covers tilted cylinder contacts against the
same plane-as-large-box fallback. The solver-facing native manifold cache
bridge is now covered through a legacy display-name facade so warm-start
impulse writeback is not accidentally disabled by compatibility type strings.
Focused gz collision, detachable joint, joint feature, and transmitted-wrench
tests now pass locally against the DART plugin. Lint now enforces the source
split by rejecting old-engine includes from non-reference DART source paths and
rejecting legacy
implementation sources outside `reference/` paths. CI, downstream migration,
and broader performance guardrail evidence remain before this phase can
complete.

The former `JointDetach` residual was reduced to native base-vs-ground support
contact stability for gz-physics' plane-as-large-box path. The fix keeps a
small active two-point support patch for tilted cylinder/plane-like-box
contacts, which preserves the expected upward motion while canceling the rim
torque that previously injected off-axis base angular `Y` motion through the
upper joint offset.

Success criteria:

- `dart/collision/` no longer exposes FCL, Bullet, or ODE as real runtime
  backend implementations.
- Any retained legacy detector class, header, factory key, or component name
  resolves to the built-in DART detector or a thin native-backed adapter.
- Selecting a legacy backend name cannot instantiate or link FCL, Bullet, or
  ODE collision code.
- Public collision APIs express DART semantics, options, filters, and results
  without engine-specific types or algorithm knobs.
- The internal layer split is explicit in code ownership: compatibility shell,
  `dart/collision/dart/` adapter, `dart/collision/native/` scene/query core, and
  optional reference harnesses outside runtime targets.
- The native core supports scalable scene updates through stable handles,
  dirty transform/shape tracking, persistent broadphase state, reusable query
  snapshots, and deterministic result ordering.
- The native core is performance-oriented: shape-specialized fast paths,
  compact native geometry data, clear cache invalidation, low-allocation hot
  loops, and independent profiling/benchmark labels for broadphase,
  narrowphase, distance, raycast, and solver-facing contact generation.
- Native support contacts are stable enough for downstream articulated
  simulation gates: manifold generation and solver-facing cache persistence do
  not inject off-axis base motion into gz-physics `JointDetach` exact-zero
  checks.
- The built-in component blueprint in `01-design.md` is satisfied without
  adding public engine-selection knobs or routing performance work through
  external backend abstractions.
- The `01-design.md` north-star layer table is satisfied: every layer has a
  narrow owner, forbidden dependencies are absent, and completion evidence is
  recorded for public API cleanliness, compatibility facades, the DART adapter,
  the native core, and optional reference harnesses.
- The `01-design.md` component design work items are satisfied for API
  cleanliness, adapter scalability, native core scalability, performance
  orientation, reference isolation, and compatibility facades.
- Public compatibility behavior needed by gz-physics is covered by DART tests.
- Documentation distinguishes stable native APIs from temporary compatibility
  wrappers.
- `04-reference-gap-analysis.md` gaps are either implemented, explicitly
  deferred with a DART-specific rationale, or converted into smaller checked
  tasks before final deletion.

Verification:

- Factory tests prove `dart`, `experimental`, and any retained legacy keys all
  resolve to native behavior.
- API review verifies installed headers and exported CMake targets expose the
  built-in DART collision surface without FCL, Bullet, or ODE types.
- Native scene/update tests cover dirty-object updates, batched queries, cache
  invalidation, and deterministic ordering.
- Solver-facing cache tests prove native manifold cache writeback works through
  native-backed compatibility facades, even when facade display strings are
  legacy names for gz-physics compatibility.
- A reduced DART or gz-focused test proves `JointDetach` base support remains
  stable without weakening the expected upward motion check.
- Benchmark labels or profiler scopes show broadphase, narrowphase, distance,
  raycast, and contact-generation costs separately.
- Link inspection shows compatibility wrappers do not link FCL, Bullet, or ODE.
- `check-collision-runtime-isolation` passes in lint and rejects old-engine
  includes or implementation sources outside the explicit reference harness.
- `pixi run -e gazebo test-gz` passes after the wrapper/adaptor cleanup.
- Repository search shows no remaining real legacy backend dispatch path in
  `dart/collision/`.
- The `01-design.md` layer acceptance gates pass for API cleanliness,
  scalability, performance hooks, reference isolation, and compatibility
  facades.
- `README.md` and `03-evidence-gates.md` show the architecture/design gate
  status on the same north-star scale used for CI, gz-physics, packaging, and
  deletion work.

## Phase 12: Performance Guardrails

Purpose: convert the one-time benchmark evidence into repeatable regression
protection.

Success criteria:

- Comparative benchmark suites can run in CI or scheduled jobs with structured
  output.
- Native remains at least as fast as the best legacy backend on the required
  benchmark set while reference backends are still available.
- Native-only baselines exist for future work after old backends are deleted.
- Larger dirty-world and simulation-style workloads are included, not only
  narrow microbenchmarks.
- Correctness tests stay the prerequisite for accepting any performance
  optimization.
- Benchmark failures drive gradual optimization work; they do not justify
  weakening feature coverage or semantics.

Verification:

- Benchmark jobs record JSON artifacts for primitive, narrow-phase, supported
  distance, raycast, batch-raycast, mesh-heavy, mixed-primitive, and
  dirty-world workloads.
- Benchmark parsers fail the job or file an explicit follow-up when native
  regresses beyond the accepted tolerance.
- Performance evidence is linked from release notes or PR descriptions for
  collision-sensitive changes.

## Phase 13: Final Runtime Cleanup And Facade Policy

Purpose: remove FCL, Bullet, and ODE implementation paths from the runtime
layer once native has CI, downstream, packaging, correctness, and performance
guardrails in place. Compatibility facades may remain only when needed for
source compatibility, and they must be native-backed.

Success criteria:

- FCL, Bullet, and ODE collision implementation source/components are deleted
  from production runtime paths.
- Legacy package dependencies are removed from normal package metadata and
  build environments.
- Compatibility facades that remain are native-backed and do not link old
  libraries.
- CMake, package exports, tests, examples, and docs describe a single native
  collision stack.
- Optional test/benchmark reference harnesses, if retained, are clearly outside
  the runtime backend layer and can be disabled by CMake.

Verification:

- Full `pixi run test-all` passes after runtime cleanup.
- gz-physics compatibility passes after runtime cleanup or after downstream
  migration lands.
- No default target links FCL, Bullet, or ODE collision libraries.
- Repository search shows old backend names only in changelog, migration notes,
  intentionally retained compatibility aliases, or optional reference
  test/benchmark harnesses.
- `check-collision-runtime-isolation` remains in lint as the local guard
  against reintroducing old runtime backend source paths.

## Phase 14: Final PR Packaging

Purpose: convert the completed north-star work into a reviewable single PR
without carrying temporary task docs.

Success criteria:

- The PR description includes the final evidence from `03-evidence-gates.md`.
- Durable architecture notes remain in onboarding docs.
- This dev-task folder is removed in the same PR.
- The branch contains no temporary benchmark artifacts or local-only notes.

Verification:

- `pixi run lint`.
- `pixi run test-all`.
- `pixi run -e gazebo test-gz`.
- Native-only CI passes with FCL, Bullet, and ODE disabled.
- Packaging/runtime dependency inspection confirms old collision libraries are
  absent from default builds.
- Optional reference-engine tests and benchmarks are documented as test-only or
  benchmark-only and can be disabled.
