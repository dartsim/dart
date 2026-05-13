# Native Collision Default Design

## Target Architecture

The target architecture has three layers:

1. Native core: algorithms and data structures in `dart/collision/native/`.
2. DART adapter: `DartCollisionDetector`, `DartCollisionGroup`, shape adapters,
   result conversion, and factory registration under `dart/collision/dart/`.
3. Compatibility facades: legacy FCL, Bullet, and ODE detector class names and
   factory keys remain only as compatibility surfaces while they delegate to, or
   are validated against, native behavior.

The default detector selection should be:

1. Prefer factory key `dart`.
2. Accept factory key `experimental` only as a backward-compatible alias.
3. Fall back to legacy backends only while native is unavailable or while a
   targeted compatibility test explicitly requests them.

## Feature Surface To Cover

Native collision must cover the DART behavior currently supplied by FCL, Bullet,
and ODE in these categories:

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

## gz-physics Compatibility

gz-physics compatibility is preserved by keeping source-compatible public names
for legacy detectors until downstream no longer needs them. Compatibility means:

- Existing gz-physics code that includes legacy detector headers still builds.
- Existing gz-physics code that subclasses legacy detector classes still has an
  ABI/API-compatible path, or a coordinated migration is documented and tested.
- DART's gz-physics integration task passes without local downstream patches.
- Legacy factory keys continue to resolve during the migration window.

## Dependency Removal Strategy

Dependency removal happens only after the evidence gates pass:

1. Keep FCL, Bullet, and ODE available in explicit reference/benchmark builds.
2. Remove them from the default DART build and runtime dependency surface.
3. Keep compatibility headers/classes if gz-physics requires names, but make
   them native-backed facades rather than external-library wrappers.
4. Remove reference backend code from normal packaging after benchmark and
   compatibility jobs no longer need it.

## Performance Strategy

Performance must be compared against the best available legacy backend for each
workload. The native detector is ready only when:

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
