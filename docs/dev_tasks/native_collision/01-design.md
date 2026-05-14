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

The native query result contract is also part of the layer boundary. Public
DART contacts must be expressed in collision object order: `collisionObject1`
and `collisionObject2` identify the pair as reported to the caller, and the
contact normal must follow that same order even when the native dispatcher calls
a canonical shape-specialized function in the opposite order. Canonical
narrowphase functions should stay shape-specialized and small; dispatch or
result assembly is responsible for flipping normals when object order differs.
This keeps the public API clean and deterministic without duplicating
narrowphase algorithms for every symmetric pair.

## Built-In Component Blueprint

This blueprint translates the layer contract into implementation rules. It is
the design target for API cleanliness, scalability, and performance-oriented
native collision work inside the single north-star PR.

This is a code design gate, not just a documentation gate. Phase 11 is not
complete until the public API, compatibility shell, DART adapter, native core,
package exports, tests, and benchmarks all follow this blueprint.

The intended component shape is reviewable at subcomponent granularity:

| Subcomponent                                | API cleanliness responsibility                                                                                                 | Scalability responsibility                                                                                                             | Performance responsibility                                                                                                 |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| Detector and factory surface                | Make `dart` the canonical detector identity; keep retained legacy keys/classes as compatibility aliases or facades only.       | Keep factory behavior independent from optional reference-engine availability.                                                         | Avoid runtime engine-selection branches in hot query paths.                                                                |
| Compatibility facades                       | Preserve source-compatible FCL, Bullet, ODE, and `experimental` names without exposing their engine types through normal APIs. | Carry no scene state; delegate to the built-in adapter so legacy names cannot create parallel worlds.                                  | Add no extra per-query dispatch beyond the facade-to-native handoff.                                                       |
| `DartCollisionGroup` adapter scene          | Hide native handles and geometry records from public headers while preserving DART object/filter/result semantics.             | Own stable object IDs, dirty membership/transform/shape sync, filter adaptation, deterministic result mapping, and cache invalidation. | Reuse synchronized scene state across collision, distance, and raycast queries instead of rebuilding native objects.       |
| Native scene and geometry tables            | Keep native data opaque and backend-independent.                                                                               | Store compact object, geometry, transform, AABB, filter, revision, and cache-generation records.                                       | Keep hot data contiguous enough for broadphase, narrowphase, and batched query traversal.                                  |
| Broadphase and query snapshots              | Expose no public broadphase selection knobs.                                                                                   | Maintain persistent broadphase state and query snapshots that can serve repeated and batched public queries.                           | Prune candidates before callback filters and narrowphase, with measurable broadphase/candidate traversal costs.            |
| Narrowphase, distance, and raycast dispatch | Keep result semantics in DART pair order rather than leaking canonical shape-dispatch order.                                   | Centralize symmetric-pair orientation so new shape pairs do not duplicate algorithms or destabilize ordering.                          | Prefer shape-specialized fast paths and reserve generic convex/mesh fallbacks for cases that need them.                    |
| Result builder and solver cache bridge      | Publish deterministic DART contacts, distances, ray hits, and warm-start data through public result types only.                | Version contact/manifold caches by stable native IDs and invalidate on object or geometry changes.                                     | Reuse contact storage and write solver impulses back to the native manifold cache without detector-type string checks.     |
| Reference harness                           | Make FCL, Bullet, and ODE access visibly test/benchmark-only through explicit reference APIs and targets.                      | Allow native-only builds to opt out completely while comparison jobs opt in.                                                           | Compare native against the best available reference engine without adding external-engine abstraction to native hot paths. |

The review rule is simple: if a change adds public backend choice, makes the
adapter own collision algorithms, makes the native core depend on legacy engine
headers, or hides reference-engine links in normal runtime targets, it moves
away from the north star even if it passes local tests.

### Public API Boundary

The public DART collision API should describe collision semantics, not backend
selection mechanics. The final shape is:

- One canonical detector identity: `dart`.
- Stable public concepts: detector, collision group, collision object, query
  options, filters, collision/distance/raycast results, and factory
  registration.
- Compatibility names: retained `experimental`, FCL, Bullet, and ODE spelling
  variants compile and route to the built-in detector. They may keep temporary
  display strings where gz-physics needs them, but they do not change the
  runtime collision implementation.
- Reference APIs: any call path that intentionally reaches FCL, Bullet, or ODE
  must be visibly named as reference/test/benchmark behavior, such as
  `createReference()` or `collision-reference-*`.

New public options should be semantic and engine-neutral. Examples of valid
public options are maximum contacts, nearest-point data, raycast hit mode,
collision filtering, and deterministic result ordering. Examples of invalid
north-star public options are FCL-specific solver modes, Bullet broadphase
selection, ODE/libccd tuning knobs, or any enum that asks the user to pick an
external collision engine.

### Adapter-Owned Scene Model

`dart/collision/dart/` is the scaling boundary between DART objects and compact
native query state. A `DartCollisionGroup` should own a persistent adapter scene
with these responsibilities:

- Map `CollisionObject` membership to stable native object handles.
- Track shape, transform, filter, and collidable-state revisions.
- Synchronize only dirty membership, transform, and geometry changes before a
  query.
- Convert DART shapes into native geometry records without exposing those
  records through public headers.
- Adapt DART body-node, self-collision, category/mask, and callback filters
  into cheap native pair rejection before narrowphase work.
- Keep deterministic mapping from native result handles back to public DART
  result objects.
- Own cache invalidation when a shape changes type, scale, mesh vertices,
  heightmap data, voxel occupancy, or soft-body point positions.

The adapter must not become a second collision engine. It should not own
broadphase algorithms, narrowphase math, reference-engine calls, or public API
policy beyond compatibility-name routing and DART result conversion.

### Native Scene And Query Core

`dart/collision/native/` owns the built-in runtime engine. Its internal data
model should be optimized for repeated queries and simulation steps:

- `NativeScene`: persistent object tables, broadphase structures, static and
  dynamic update queues, filter data, and scene-wide generation counters.
- `NativeGeometry`: compact shape-specific data, derived AABBs, support data,
  mesh or heightfield acceleration structures, and mutation revision numbers.
- `QuerySnapshot`: read-only view of synchronized scene state for one public
  query or batch of queries.
- `QueryContext`: reusable scratch memory, temporary candidate lists,
  narrowphase work buffers, profiler scopes, and per-query limits.
- `ResultBuilder`: deterministic merge/sort, pair-order normal orientation,
  contact/manifold assembly, distance result assembly, raycast hit ordering,
  and public-adapter metadata.

This split keeps public API stability independent from native implementation
choices. The native core can add a better broadphase, manifold cache,
parallel candidate traversal, or SIMD-oriented shape data without exposing new
engine choices to users.

### Query Lifecycle

Every public collision, distance, and raycast query should follow the same
high-level lifecycle:

```text
DART caller
  -> public query options and filters
  -> compatibility facade, if a legacy name was used
  -> DartCollisionGroup dirty sync
  -> native scene snapshot
  -> broadphase or query candidate pruning
  -> native filter checks
  -> shape-specialized narrowphase/distance/raycast
  -> deterministic native result assembly
  -> DART result conversion in public pair order
```

The lifecycle should be measurable and testable. The adapter owns sync and
conversion costs. The native core owns candidate generation and algorithmic
costs. Result orientation is explicit: public contacts are ordered by the
reported collision object pair, and dispatch wrappers flip normals when a
canonical shape-pair function runs in the opposite order.

### Scalability Design

Scalability means public queries should stay efficient as worlds grow, move,
and mutate. The north-star design requires:

- Persistent broadphase state rather than rebuilding candidate structures per
  query.
- Stable object handles so caches survive between simulation steps and are
  invalidated deliberately.
- Incremental transform and AABB updates for dirty objects.
- Explicit geometry-update paths for dynamic meshes, soft meshes, heightmaps,
  and voxel grids.
- Separate cheap filter data from expensive callback filters so most rejected
  pairs never reach narrowphase.
- Query snapshots that allow collision, distance, and raycast batches to share
  synchronization and broadphase state.
- Deterministic result ordering after broadphase pruning, batching, or future
  parallel traversal.
- Cache lifetimes tied to scene/object/geometry generations, not raw DART
  object addresses.

The public API should not change when these scaling mechanisms improve. A
caller should see the same `DartCollisionDetector` semantics while the adapter
and native core become more incremental internally.

### Performance-Oriented Internals

The native component should be easy to optimize without reopening API design.
Performance-sensitive implementation rules are:

- Keep hot data compact and native-owned: shape type tags, transforms, AABBs,
  filter masks, and geometry payloads should be reachable without public DART
  object traversal inside per-pair loops.
- Use a shape-specialized dispatch matrix for common DART pairs and reserve
  generic convex or mesh paths for cases that need them.
- Reuse canonical symmetric-pair algorithms and put object-order contact normal
  fixes in thin dispatch or result-builder wrappers.
- Reuse scratch storage for candidate lists, support queries, raycast hits,
  contact points, and manifold generation.
- Cache derived geometry and acceleration data behind explicit revision checks.
- Keep solver-facing contact/manifold caches versioned by stable native IDs so
  shape replacement cannot reuse stale impulses.
- Treat solver cache writeback as native cache metadata, not detector
  display-name behavior. Compatibility facades may report legacy display
  strings for gz-physics, but any native `CachedContact` attached to a public
  `Contact` must receive the applied solver impulses.
- Expose profiler and benchmark labels for adapter sync, broadphase update,
  candidate traversal, filtering, narrowphase, distance, raycast, contact
  generation, manifold update, result merge, and DART result conversion.
- Keep FCL/Bullet/ODE comparisons in optional reference tests and benchmarks so
  native hot paths never pay for external-engine abstraction.

Performance acceptance still follows correctness. A benchmark win does not
justify weaker contact semantics, unstable normals, nondeterministic ordering,
or skipped dynamic-geometry invalidation.

### Architecture Evidence Plan

Every design axis needs evidence before the north-star PR can close:

- API cleanliness evidence: factory tests, C++ facade tests, Python binding
  smoke tests, installed-header/package-export inspection, and source searches
  proving retained legacy names route to native or explicit reference APIs.
- Scalability evidence: public DART adapter tests for dirty membership,
  transform and shape updates, dynamic-geometry invalidation, filter
  adaptation, stable cache IDs, deterministic ordering, pair-order normals,
  and collision/distance/raycast query reuse.
- Performance evidence: native and public-adapter benchmark JSON with labels
  for sync, broadphase, candidate traversal, filtering, narrowphase, distance,
  raycast, contact/manifold generation, result merge, and DART conversion.
- Reference isolation evidence: native-only configure/build/install/wheel
  probes without FCL, Bullet, ODE, or libccd runtime links, plus opt-in
  reference jobs for correctness comparisons and benchmark guidance.
- Compatibility evidence: gz-physics and downstream package smokes showing
  legacy names compile and run through the built-in detector, with any
  temporary facade behavior documented in `05-downstream-migration.md`.

## North-Star Layer Design

This table is the review contract for the built-in collision component. A row
is not complete until code, tests, package metadata, and benchmark/profiling
evidence all match the intended ownership boundary.

| Layer                   | Owns                                                                                                                                                                                                                                         | Must not own                                                                                                                                                   | Completion evidence                                                                                                                                                                       |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Public DART API         | `CollisionDetector`, `CollisionGroup`, query options, filters, results, canonical factory keys, and stable DART collision semantics.                                                                                                         | FCL, Bullet, ODE types; external-engine algorithm modes; user-visible backend selection knobs.                                                                 | Public headers and bindings expose `dart` as canonical; retained keys and Python/C++ names construct/report the built-in detector; result semantics are covered by tests.                 |
| Compatibility shell     | Temporary legacy headers, class names, package component names, and factory aliases needed by gz-physics or source-compatible downstreams.                                                                                                   | Collision algorithms, external-engine state, hidden package links, or behavior that differs from the built-in detector except documented compatibility quirks. | Legacy names compile without FCL/Bullet/ODE headers, route to the built-in detector, and pass gz-physics/downstream smoke coverage.                                                       |
| DART adapter            | `CollisionObject` membership, `ShapeFrame` access, shape/result/filter adaptation, persistent adapter scene state, stable IDs, dirty sync, deterministic result conversion, and factory registration.                                        | Narrowphase algorithms, broadphase data structures, external backend calls, or public API churn for internal optimization.                                     | Collision, distance, raycast, filter, dynamic-shape invalidation, cache invalidation, and deterministic ordering tests exercise the public DART path.                                     |
| Native scene/query core | Compact geometry, object handles, broadphase state, snapshots/query contexts, candidate traversal, narrowphase dispatch, distance, raycast, CCD/sweeps, contact/manifold generation, scratch reuse, cache invalidation, and profiler labels. | DART public object ownership, gz-physics compatibility policy, package metadata, or legacy backend dependencies.                                               | Native tests cover the feature surface; benchmarks expose adapter sync, broadphase, candidate traversal, narrowphase, distance, raycast, contact generation, and result conversion costs. |
| Reference harness       | Optional FCL/Bullet/ODE correctness comparisons and comparative benchmarks.                                                                                                                                                                  | Runtime collision target links, default package dependencies, installed compatibility headers, or public backend selection.                                    | CMake opt-in jobs build `collision-reference-*` targets; native-only builds opt out cleanly; benchmark JSON compares native to the best enabled reference engine.                         |

The design has four invariants:

- One public implementation: every normal factory key, package component, C++
  compatibility class, and Python compatibility name resolves to the built-in
  detector. Explicit reference APIs are the only path to old engines.
- One data direction: DART objects and compatibility policy stay outside the
  native core; the adapter passes compact, versioned native state inward.
- One scalability boundary: adapter-owned scenes synchronize dirty state
  incrementally and keep persistent broadphase/query data across public calls.
- One optimization surface: performance work adds native capabilities,
  profiler labels, and benchmarks without exposing engine-specific knobs in
  public APIs.
- One contact convention: native and public tests must pin contact normals,
  points, and depths to DART pair-order semantics so solver-facing downstream
  behavior is not an accident of broadphase pair order.

## Component Design Work Items

These work items are the implementation form of the architecture. They should
be tracked through `README.md`, `02-milestones.md`, and
`03-evidence-gates.md` as the PR moves toward completion.

| Design area             | Required implementation shape                                                                                         | Evidence needed before completion                                                                                  |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| API cleanliness         | Canonical `dart` detector; legacy names are native-backed compatibility facades; reference engines use explicit APIs. | Factory, C++, Python, package-export, installed-header, source-search, and downstream smoke evidence.              |
| Adapter scalability     | `DartCollisionGroup` owns persistent scene state with stable IDs, dirty sync, filter adaptation, and result mapping.  | Collision, distance, raycast, dynamic-shape invalidation, cache invalidation, and deterministic ordering tests.    |
| Native core scalability | `dart/collision/native/` owns compact geometry, broadphase/query snapshots, dispatch, caches, scratch, and profiling. | Native feature tests, public adapter tests, source-boundary linting, and benchmark/profiler labels.                |
| Performance orientation | Shape-specialized fast paths, no external-engine hot-loop dispatch, reusable query memory, and benchmark JSON guards. | Native-vs-reference benchmarks while references exist, plus native-only public-adapter regression benchmarks.      |
| Reference isolation     | FCL/Bullet/ODE live only in optional reference correctness tests and comparative benchmark targets.                   | CMake opt-out builds, native-only install/wheel probes, link inspection, wheel verifier, and CI artifact evidence. |
| Compatibility facade    | gz-physics-required names compile and route to native without exposing old runtime engines.                           | gz-physics focused/full tests and documented migration/removal gates.                                              |

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
  invalidation for object removal and dynamic geometry mutation. Pair-order
  contact normal tests must cover direct native dispatch and the public DART
  adapter path.
- Performance gate: native hot paths use compact geometry data,
  shape-specialized dispatch, persistent broadphase state, reusable scratch,
  contact/manifold caches with explicit lifetimes, and profiler/benchmark
  labels for adapter sync, broadphase update, candidate traversal,
  narrowphase, distance, raycast, contact generation, and result conversion.
  Symmetric-pair handling should reuse canonical narrowphase functions and keep
  object-order normal fixes in thin dispatch/result wrappers.
- Reference isolation gate: FCL, Bullet, and ODE are reachable only from
  explicit reference correctness tests and comparative benchmarks. Native-only
  runtime, dartpy, wheel, installed-package, and downstream compatibility paths
  must not link or advertise old collision runtime engines. Lint must also
  guard the source boundary so non-reference DART source paths cannot include
  old-engine or reference-backend headers.
- Compatibility facade gate: gz-physics-required legacy names still compile
  during the migration window. Factory selection and Python access route to the
  canonical built-in `dart` detector; direct C++ legacy facade objects may keep
  legacy display type strings when gz-physics depends on them, but their
  algorithms and scene state still come from `DartCollisionDetector` unless the
  call site is an explicitly named reference test/benchmark harness.

## Current Design Status

The codebase already implements the core shape of this design: native geometry
and query infrastructure live in `dart/collision/native/`, the public DART
adapter lives in `dart/collision/dart/`, the `dart` factory key is canonical,
retained factory aliases route to `DartCollisionDetector`, Python detector
compatibility names are native-backed, and normal package/wheel surfaces no
longer carry old collision dependencies. Reference tests and benchmarks now
call explicit `createReference()` methods on the FCL, Bullet, and ODE detector
classes when they intentionally need old-engine comparisons. Direct public C++
legacy detector `create()` entry points now return native-backed facades over
`DartCollisionDetector`; those facade objects may keep legacy display type
strings for downstream compatibility, but ordinary source code that selects an
old detector class still lands on native collision algorithms and state.

The component boundary has been split: retained package component names
`collision-fcl`, `collision-bullet`, and `collision-ode` are native-backed
interface facades, while old-engine libraries/components are explicitly named
`collision-reference-fcl`, `collision-reference-bullet`, and
`collision-reference-ode`. Installed FCL, Bullet, and ODE detector headers are
compatibility facades over `DartCollisionDetector` in both native-only and
reference-enabled installs, while old-engine object/type/detail headers are no
longer installed on the public legacy paths. Source-tree legacy headers now
match that boundary: top-level FCL, Bullet, and ODE detector/group,
PascalCase, All, and component headers are native-backed compatibility
facades, and old-engine implementation headers/sources live under explicit
`reference/` paths used by reference tests and benchmarks. Runtime source
isolation is now checked by lint so non-reference DART source paths cannot
include old-engine or reference-backend headers, and legacy implementation
sources cannot move back outside `reference/` paths. The native dispatcher now
has focused tests for pair-order contact normals across direct narrowphase,
snapshot collision, and public DART collision group paths; custom mesh-plane
stacked parallel-cylinder, and axial cylinder-cap/large-box gz contact
regressions are reduced and fixed on the DART side. The solver-facing native
manifold cache contract is now also tested through a legacy display-name
facade: a compatibility detector may report a legacy type string while the
attached native `CachedContact` still receives normal and friction impulse
writeback. The remaining design gaps are CI and packaging evidence at matrix
scale, downstream migration/deprecation evidence, the focused gz-physics
`JointDetach` exact-zero velocity residual, and broader recurring
correctness/performance guardrails across the public DART adapter and native
core paths. The completed PR must make it impossible for ordinary DART
collision runtime selection to instantiate or link FCL, Bullet, or ODE.

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
  profiling scopes, benchmark-facing statistics, and canonical shape-pair
  functions whose results are oriented by dispatch/result wrappers when the
  public pair order differs.
- Reference harnesses: optional tests and benchmarks that compare public DART
  behavior against FCL, Bullet, or ODE. These harnesses must not be reachable
  from default runtime targets.

The dependency rule is one-way: public and compatibility surfaces call into the
DART adapter; the DART adapter calls the native core; the native core never
includes old runtime backend headers. The lint-time runtime isolation check is
the local enforcement for that source boundary. This is what keeps the API
clean while still allowing the native implementation to use specialized
internal data structures.

## Installed Package And API Boundary

The installed package surface is part of the architecture contract. Downstream
users should see a single normal collision stack, not a menu of runtime
backends:

- `find_package(DART)` should default to the `dart` component and the built-in
  detector. It should not auto-add FCL, Bullet, or ODE collision components.
- Installed CMake component files, pkg-config metadata, and default runtime
  libraries should not advertise external-engine collision libraries in
  native-only builds.
- Retained legacy component names such as `collision-fcl`,
  `collision-bullet`, and `collision-ode` must be native-backed compatibility
  facades. External-engine comparison targets must use explicit
  `collision-reference-*` names and must not silently select an external
  runtime backend.
- Installed legacy detector headers must compile without FCL, Bullet, or ODE
  headers and construct native-backed detector facades, even when the build
  tree has reference engines enabled for tests and benchmarks.
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
