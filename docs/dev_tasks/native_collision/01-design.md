# Native Collision Single Built-In Design

## Target Architecture

The north-star architecture has one collision implementation and a thin
compatibility shell:

1. Native core: algorithms and data structures in `dart/collision/native/`.
2. DART adapter: `DartCollisionDetector`, `DartCollisionGroup`, shape adapters,
   result conversion, and factory registration under `dart/collision/dart/`.
3. Compatibility adapters: any legacy FCL, Bullet, ODE, or experimental class
   names, headers, factory keys, or component names that must remain for
   gz-physics/source compatibility are wrappers over the built-in DART detector.

The final detector selection rule is:

1. Factory key `dart` creates the built-in DART detector.
2. Factory key `experimental`, if retained, is only a compatibility alias for
   the built-in DART detector.
3. Any retained legacy factory keys such as FCL, Bullet, or ODE names must also
   resolve to the built-in DART detector or a thin native-backed adapter.
4. No selected DART collision "backend" may instantiate or link an external
   FCL, Bullet, or ODE collision implementation in the final PR state.

`dart/collision/` should no longer present a real multi-backend abstraction in
the completed PR. The stable public surface can still be abstract enough for
DART API compatibility, but the only implementation behind it is the built-in
detector. Legacy wrappers exist to preserve downstream source compatibility,
not to choose a different runtime engine.

## Built-In Component Architecture

The built-in detector should be designed as a layered component, not as another
backend plugged into the old backend-selection model:

1. Public collision API: stable DART-facing detector, group, query option,
   filter, and result types under `dart/collision/`. This surface should use
   DART concepts only; it must not expose FCL, Bullet, ODE, or native-core
   implementation details.
2. Compatibility shell: retained legacy detector names, headers, component
   names, and factory aliases. These names canonicalize to the built-in DART
   detector and exist only to keep downstream source compatible during the
   migration window.
3. DART adapter layer: `dart/collision/dart/` owns conversion between
   `CollisionObject`, `ShapeFrame`, shape data, filters, and DART result
   formats. It also owns factory registration and compatibility-name routing.
   It should not own narrowphase, broadphase, or acceleration algorithms.
4. Native scene/query core: `dart/collision/native/` owns geometry
   representation, transform snapshots, broadphase state, narrowphase
   dispatch, distance/raycast/contact algorithms, contact persistence, result
   assembly helpers, and profiling hooks.
5. Optional reference harnesses: tests and benchmarks may adapt public DART
   queries to FCL, Bullet, or ODE for correctness and performance comparison.
   These harnesses must live outside the runtime collision layer and be
   possible to disable through CMake.

Layer dependencies should point inward and downward: compatibility facades call
the DART adapter, the DART adapter calls the native core, and the native core
does not depend on legacy runtime backend components. External reference
engines, if enabled, are consumed only by test and benchmark targets.

## Built-In Layer Contract

The runtime query path in the completed PR should be explicit and boring:

```text
public DART collision API
  -> optional compatibility facade
  -> dart/collision/dart adapter scene
  -> dart/collision/native scene/query core
```

Each layer has a narrow ownership contract:

- Public API layer: owns stable DART names, options, filters, result types, and
  factory keys. It must expose DART collision semantics, not engine selection.
  The canonical key is `dart`; retained legacy keys are compatibility aliases.
- Compatibility facade layer: owns source-compatible legacy names that
  downstreams such as gz-physics still include. These facades may preserve
  headers, component names, class names, or factory keys, but they do not own
  collision state or algorithms and they cannot select an external engine.
- DART adapter layer: owns `CollisionObject` membership, `ShapeFrame` access,
  transform and shape revision synchronization, DART filter adaptation, result
  conversion, deterministic ordering, and stable scene/cache IDs. The adapter
  may use opaque native user data to map results back to DART objects, but it
  should not implement broadphase or narrowphase algorithms.
- Native scene/query core: owns compact geometry data, object handles,
  broadphase state, query snapshots, narrowphase dispatch, distance, raycast,
  contact/manifold generation, cache invalidation, and profiling. It should not
  include legacy backend headers or depend on FCL, Bullet, or ODE runtime
  components.
- Reference harness layer: owns optional FCL/Bullet/ODE comparison tests and
  benchmarks. It is outside the runtime stack and can be disabled by CMake
  without changing normal DART collision behavior.

The adapter scene is the main scalability boundary. A `DartCollisionGroup`
owns persistent native scene state for its membership and synchronizes only
dirty membership, transform, and shape data before queries. This keeps the
public API stable while allowing the native core to evolve toward better
broadphase traversal, batch queries, and low-allocation hot paths.

## Layer Acceptance Gates

These gates define what "clean built-in collision layer" means for this PR.
They are completion criteria for the collision abstraction cleanup, not
follow-up nice-to-haves.

- API cleanliness gate: public headers, factory keys, Python bindings, CMake
  package exports, and compatibility headers present `dart` as the canonical
  collision detector. Retained `experimental`, FCL, Bullet, and ODE spellings
  must be aliases or native-backed compatibility facades, and public query
  options/results must avoid engine-specific types and knobs.
- Scalability gate: the DART adapter owns persistent scene state with stable
  handles, dirty membership/transform/shape synchronization, reusable
  broadphase/query snapshots, deterministic result ordering, and clear cache
  invalidation for object removal and dynamic geometry mutation.
- Performance gate: native hot paths use compact geometry data,
  shape-specialized dispatch, persistent broadphase state, reusable scratch,
  contact/manifold caches with explicit lifetimes, and profiler/benchmark
  labels for adapter sync, broadphase update, candidate traversal,
  narrowphase, distance, raycast, contact generation, and result conversion.
- Reference isolation gate: FCL, Bullet, and ODE are reachable only from
  explicit reference correctness tests and comparative benchmarks. Native-only
  runtime, dartpy, wheel, installed-package, and downstream compatibility paths
  must not link or advertise old collision runtime engines.
- Compatibility facade gate: gz-physics-required legacy names still compile
  during the migration window, but construction, factory selection, and Python
  access all route to the built-in detector unless the call site is an
  explicitly named reference test/benchmark harness.

## Current Design Status

The codebase already implements the core shape of this design: native geometry
and query infrastructure live in `dart/collision/native/`, the public DART
adapter lives in `dart/collision/dart/`, the `dart` factory key is canonical,
retained factory aliases route to `DartCollisionDetector`, Python detector
compatibility names are native-backed, and normal package/wheel surfaces no
longer carry old collision dependencies.

The remaining design gap is direct C++ legacy surface cleanup. The old FCL,
Bullet, and ODE detector classes, headers, and component targets still contain
real reference-engine implementations for comparison work. Before Phase 11 can
complete, those surfaces must either become thin native-backed compatibility
facades or move behind explicitly named reference-only test/benchmark targets.
The completed PR must make it impossible for ordinary DART collision runtime
selection to instantiate or link FCL, Bullet, or ODE.

## Code Ownership Map

The final code layout should make the layer boundary visible during review:

- `dart/collision/`: stable public DART collision concepts plus temporary
  compatibility facades. This layer may define detector/group/query/filter and
  result APIs, but it should not expose FCL, Bullet, ODE, or native-core
  implementation details.
- `dart/collision/dart/`: DART adapter implementation. This layer owns
  `DartCollisionDetector`, `DartCollisionGroup`, adapter-owned scene sync,
  DART shape/result/filter conversion, factory registration, legacy factory
  alias routing, deterministic ordering, and compatibility-name behavior.
- `dart/collision/native/`: built-in scene/query engine. This layer owns
  compact geometry, object handles, broadphase state, narrowphase dispatch,
  distance, raycast, contact/manifold generation, cache invalidation,
  profiling scopes, and benchmark-facing statistics.
- Reference harnesses: optional tests and benchmarks that compare public DART
  behavior against FCL, Bullet, or ODE. These harnesses must not be reachable
  from default runtime targets.

The dependency rule is one-way: public and compatibility surfaces call into the
DART adapter; the DART adapter calls the native core; the native core never
includes old runtime backend headers. This is what keeps the API clean while
still allowing the native implementation to use specialized internal data
structures.

## Installed Package And API Boundary

The installed package surface is part of the architecture contract. Downstream
users should see a single normal collision stack, not a menu of runtime
backends:

- `find_package(DART)` should default to the `dart` component and the built-in
  detector. It should not auto-add FCL, Bullet, or ODE collision components.
- Installed CMake component files, pkg-config metadata, and default runtime
  libraries should not advertise old collision component targets or old
  collision libraries in native-only builds.
- If a legacy component name remains for source compatibility, it must be a
  compatibility facade over the built-in detector or an explicitly named
  reference-only target. It must not silently select an external runtime
  backend.
- Default package environments, including wheel build environments, should not
  carry FCL, Bullet, ODE, or their transitive collision packages. A separate
  reference environment may carry them for comparison tests and benchmarks.
- Reference-engine build options may report whether FCL, Bullet, and ODE were
  enabled, but `OFF` option state in installed metadata is state reporting, not
  a runtime dependency.
- The public API review for the final PR must cover both headers and package
  exports: users should be guided toward `dart`, and legacy names should read
  as temporary compatibility.

## API Cleanliness Rules

The cleaned-up collision API should make the built-in detector feel like the
normal DART collision system, not a renamed experimental backend:

- `dart` is the canonical detector key. `experimental` and retained legacy
  backend keys are aliases or compatibility names, not separate engines.
- Public query options describe requested semantics, such as contact count,
  distance data, raycast mode, filtering, and nearest-point behavior. They
  should not ask callers to choose engine-specific algorithm modes.
- Public result types carry deterministic DART semantics for contact normal,
  point, penetration depth, distance, raycast hit ordering, object identity,
  and filter outcomes.
- New extension points should be capability- or option-based where possible,
  instead of reintroducing a virtual backend stack.
- Installed headers and exported CMake targets should make native collision the
  default surface; compatibility headers are explicitly temporary facades.
- CMake components and exported targets should not imply that FCL, Bullet, or
  ODE are selectable runtime collision implementations in the final PR. If a
  legacy component name remains for source compatibility, it should route to a
  native-backed facade or to an explicit test/benchmark reference target.

## Scalable Native Core Design

The native core should scale from small unit-test worlds to large dirty-world
simulation workloads without requiring callers to rebuild the world manually:

- Keep scene state persistent at the adapter boundary: one DART collision group
  synchronizes into one native world/query scene instead of constructing native
  objects from scratch for every public query.
- Use stable object handles and versioned shape/transform state so group
  updates can be incremental.
- Separate static, kinematic, and dynamic update paths where that materially
  reduces broadphase churn.
- Keep broadphase data structures persistent across simulation steps and update
  only dirty bounds.
- Support batched collision, distance, and raycast queries through one scene
  snapshot so repeated queries do not redo expensive synchronization work.
- Treat dynamic-vertex and dynamic-element shapes as explicit geometry-update
  cases so soft meshes and other deforming geometry cannot leave stale native
  acceleration data behind.
- Keep filter evaluation close to pair generation: cheap category/mask checks
  should reject pairs before callback filters, and callback filters should run
  before narrowphase work.
- Keep allocation-heavy work out of per-pair hot loops; reuse scratch buffers
  and contact/manifold storage through explicit cache lifetimes.
- Preserve deterministic result ordering after broadphase and batched query
  parallelism are introduced.

## Performance-Oriented Design

Performance is part of the architecture, not a cleanup afterthought. The native
core should be structured so optimization can happen without changing public
APIs:

- Put shape-specialized fast paths before generic convex or mesh fallbacks for
  common DART workloads.
- Keep broadphase, narrowphase, distance, and raycast timing independently
  measurable through profiler scopes and benchmark labels.
- Avoid external-engine virtual dispatch in hot loops; dispatch should resolve
  to native algorithms and compact native geometry data.
- Cache derived geometry data, AABBs, support mappings, mesh acceleration
  structures, and contact manifolds behind clear invalidation rules.
- Make performance-sensitive tolerances and iteration limits explicit native
  configuration where needed, but keep defaults stable and tested.
- Add benchmark and profiler boundaries at the same layer boundaries as the
  implementation: adapter sync, broadphase update, candidate traversal,
  narrowphase, result merge, and DART result conversion.
- Treat comparative benchmarks as optimization guidance and native-only
  benchmarks as long-term regression protection after old runtime backends are
  removed.

## Architecture Review Checklist

Use this checklist before marking the collision abstraction cleanup complete:

- API cleanliness: public headers, factory keys, and installed package exports
  present `dart` as the canonical detector; retained legacy names are visibly
  compatibility facades or explicit reference-only surfaces.
- Source compatibility: gz-physics-required names still compile, but selecting
  those names cannot instantiate an external runtime engine.
- Layering: `dart/collision/dart/` owns DART adaptation and compatibility
  routing; `dart/collision/native/` owns algorithms and query state; reference
  engines are absent from default runtime target links.
- Scalability: public DART queries use persistent scene state, stable handles,
  dirty transform/shape synchronization, persistent broadphase data, reusable
  query snapshots, and deterministic result ordering.
- Performance: hot paths avoid external-engine virtual dispatch and repeated
  allocation; benchmark/profiler labels expose adapter sync, broadphase,
  candidate traversal, narrowphase, distance, raycast, contact generation, and
  result conversion costs.
- Correctness: every feature path kept in the public API has native tests, and
  reference comparisons are optional gates rather than runtime dependencies.

## Feature Surface To Cover

Native collision must cover the DART behavior historically supplied by FCL,
Bullet, and ODE in these categories:

- Collision queries with stable contact normal, point, penetration depth, and
  contact ordering semantics needed by constraints.
- Signed distance queries with optional nearest points and filtering.
- Raycast queries with closest-hit and all-hit behavior where DART exposes it.
- Shape adapters for primitive, convex, mesh, heightmap, ellipsoid, cone,
  multi-sphere, and other DART shapes used by tests, examples, and gz-physics.
- Collision filtering, self-collision filtering, category/mask filtering, and
  callback filters.
- Persistent contact/manifold data needed for solver warm-starting and stable
  stacking.
- Broadphase behavior for single-group and group-vs-group queries at DART
  simulation scales.

## Quality And Sequencing Policy

The north-star work is sequenced by risk:

1. Feature coverage comes first: native must implement the DART collision
   surface users rely on.
2. Correctness is the permanent gate: every supported feature needs tests that
   validate native semantics, including reference comparisons where useful.
3. Performance optimization comes after correctness: native should eventually
   beat all reference engines for required workloads, but performance work must
   not weaken correctness or feature coverage.

FCL, Bullet, and ODE may remain as optional reference engines for correctness
tests and benchmarks. They must not remain as runtime collision backends. Their
allowed scope is:

- Test-only reference comparisons for contact, distance, raycast, filtering,
  and solver-facing semantics.
- Benchmark-only comparisons while native is being optimized to beat the best
  legacy engine.
- CMake option paths that allow reference engines to be enabled for comparison
  jobs and disabled for native-only builds.

The completed PR must prove both modes:

- Native-only mode: DART configures, builds, tests, imports through dartpy, and
  runs gz-physics-compatible code without FCL, Bullet, or ODE.
- Reference mode: optional FCL/Bullet/ODE test and benchmark jobs can still run
  to guard correctness and performance while the migration is active.

## gz-physics Compatibility

gz-physics compatibility is preserved by keeping source-compatible public names
for legacy detectors until downstream no longer needs them. These names are
compatibility adapters, not backend selections. Compatibility means:

- Existing gz-physics code that includes legacy detector headers still builds.
- Existing gz-physics code that subclasses legacy detector classes still has an
  ABI/API-compatible path, or a coordinated migration is documented and tested.
- DART's gz-physics integration task passes without local downstream patches.
- Legacy factory keys continue to resolve during the migration window, but they
  resolve to the built-in DART detector or a native-backed adapter.

## Dependency Removal Strategy

Dependency removal happens only after the evidence gates pass:

1. Move FCL, Bullet, and ODE into optional test/benchmark reference-engine
   scope, controlled by CMake options that native-only builds can disable.
2. Remove them from the default DART build and runtime dependency surface.
3. Collapse the `dart/collision/` backend abstraction so the final codebase has
   one built-in runtime detector.
4. Keep compatibility headers/classes if gz-physics requires names, but make
   them native-backed facades rather than external-library wrappers.
5. Delete production runtime backend source/components once benchmark evidence
   and compatibility adapters cover the migration path. Optional reference
   harnesses for tests/benchmarks may remain outside the runtime backend layer.

## Performance Strategy

Performance must first be compared against the best available legacy backend
for each workload. The performance bar is deliberately high: native should
eventually beat Bullet, FCL, and ODE on required DART workloads. The path to
that bar is gradual and evidence driven: first preserve correctness, then
optimize bottlenecks shown by benchmarks. After old runtime backends are
deleted, the same benchmark coverage becomes native-only regression protection.
The native detector is ready only when:

- Narrowphase cases are at least as fast as the best backend.
- Broadphase-heavy scenarios are at least as fast as the best backend.
- Distance queries are at least as fast as the best backend for required DART
  workloads.
- Raycast workloads are at least as fast as Bullet/ODE where those backends
  support raycast.
- Mesh-heavy and mixed-shape scenarios do not regress simulation throughput.

Any exception must be documented with a DART-specific rationale and an issue or
follow-up milestone; it is not a silent pass.

## Documentation Strategy

During active work, detailed progress and evidence live in this folder. After
completion, only durable architecture decisions move to onboarding docs, and
this folder is deleted in the completing PR.
