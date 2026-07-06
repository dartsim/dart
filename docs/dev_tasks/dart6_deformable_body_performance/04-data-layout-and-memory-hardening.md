# WP-DB.06 data layout and memory hardening

Captured on 2026-07-05 from local branch
`js/dart6-deformable-performance`.

## Why this packet exists

The current soft-body runtime is not cache-friendly:

- `SoftBodyNode` owns `std::vector<PointMass*>`.
- Each `PointMass` is individually allocated.
- Hot loops in `SoftBodyNode` pointer-chase point masses and connected
  neighbors.
- Point-mass position, velocity, acceleration, force, and cached implicit
  terms live as an array of objects rather than phase-oriented contiguous
  arrays.

That structure blocks efficient single-core cache use and makes SIMD a
second-order optimization. Before a `dart/simd/` kernel can be justified, the
data accessed by a point-mass phase needs a stable contiguous shape or an
internal SoA scratch/cache with deterministic refresh semantics.

## New evidence tool

This packet adds `tests/benchmark/integration/soft_body_headless.cpp`, wired as
the `soft_body_headless` CMake target and `pixi run bm-soft-body-headless`.

The driver:

- loads a named soft-body SKEL scene or custom URI,
- sets `World::setNumSimulationThreads()` from `THREADS`,
- prints deterministic skeleton and point-mass checksums at checkpoints,
- reports elapsed stepping time,
- dumps the built-in text profiler summary when `DART_BUILD_PROFILE=ON`.

Example commands:

```bash
./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless soft_bodies 200 100
THREADS=4 ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless soft_bodies 200 100
```

## Initial profile smoke

Scene: `soft_bodies`, 200 steps, checkpoint 100, scalar Release build
(`DART_ENABLE_SIMD=OFF`), CPU scaling enabled.

| Threads | Elapsed ms | Steps/s | Step-200 checksum |
| ---: | ---: | ---: | --- |
| 1 | 316.025 | 632.9 | matched `THREADS=4` |
| 4 | 193.915 | 1031.4 | matched `THREADS=1` |

Top inclusive profile rows for `THREADS=1`:

| Scope | Share |
| --- | ---: |
| `CollisionGroup update objects` | 83.3% |
| `SoftBodyNode::updateBiasForce` | 5.58% |
| `SoftBodyNode::updateArtInertia` | 2.06% |
| `SoftBodyNode::updateAccelerationFD` | 0.84% |

The first soft-body-specific CPU lane is therefore not "write random SIMD in
PointMass." The evidence points to:

1. reduce collision-object update cost for soft meshes or avoid refreshing
   unchanged soft collision data,
2. make `updateBiasForce` and `updateArtInertia` point-mass phases contiguous
   and allocation-free,
3. only then add SIMD kernels where scalar-vs-SIMD checksums stay within the
   approved FP contract.

## First implementation slice

The first WP-DB.06 implementation keeps the public `SoftBodyNode` and
`SoftMeshShape` APIs intact, but removes avoidable work from the FCL soft-mesh
hot path:

- `createSoftMesh()` now builds the FCL BVH with shared vertices and triangle
  indices instead of duplicating three vertices for every face.
- FCL's previous-vertex buffer is allocated by a one-time update at collision
  geometry creation, not on the first simulation collision update.
- `FCLCollisionObject::updateEngineData()` updates one FCL vertex per point
  mass instead of rewriting every triangle.
- The expensive FCL BVH refit is skipped when the soft mesh's local point-mass
  positions are unchanged; the object transform and AABB still update every
  collision pass, so rigid motion remains represented.
- `SoftMeshShape::update()` now iterates the existing point-mass vector
  directly when callers explicitly refresh the Assimp mesh.

Scene: `soft_bodies`, 200 steps, checkpoint 100, scalar Release build
(`DART_ENABLE_SIMD=OFF`), same checksum rows as the initial profile smoke.

| Threads | Before elapsed ms | After elapsed ms | After steps/s | Refit calls |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 316.025 | 198.329 | 1008.4 | 509 / 1000 |
| 4 | 193.915 | 165.997 | 1204.8 | 509 / 1000 |

The step-200 checksum values matched before and after the optimization and
matched between `THREADS=1` and `THREADS=4` in these profile runs.

Remaining profile signal after this slice:

- `FCLCollisionObject::updateSoftMesh refit` remains dominant when local
  vertices actually deform: about 75% inclusive share for `THREADS=1` and 74%
  for `THREADS=4` in the 200-step `soft_bodies` run.
- The vertex scan/update portion is no longer the limiter; future work should
  either reduce how often soft meshes need full FCL refits, use a cheaper
  collision representation for inactive/non-contact soft regions, or move to
  the tracked adaptive-contact lane.
- This slice does not solve `std::vector<PointMass*>` pointer chasing,
  per-point heap ownership, or SIMD layout. Those remain separate layout work,
  ideally coordinated with `origin/dart6-memory-hardening` once that branch is
  available on the current release base.
- This slice is FCL-specific because current DART 6 soft collision already goes
  through FCL. It is not the desired end state. The native-first collision lane
  is tracked separately in `05-native-collision-deformable-lane.md`.

## Scalar parent-term cache slice

The next WP-DB.06 implementation keeps the public `SoftBodyNode` and
`PointMass` APIs intact, but removes repeated parent soft-body lookups from the
hot point-mass recursive dynamics loops:

- `SoftBodyNode::updateArtInertia()` now computes the damping coefficient and
  vertex spring stiffness once per soft body, then calls a protected
  cached-parameter `PointMass::updateArtInertiaFD()` overload.
- `SoftBodyNode::updateBiasForce()` now computes parent angular velocity,
  local gravity, vertex stiffness, edge stiffness, and damping once per soft
  body, then calls a protected cached-parameter
  `PointMass::updateBiasForceFD()` overload.
- `PointMass` still owns the per-point dynamics formulas. The old protected
  methods remain and delegate through the cached overloads, so existing internal
  call sites keep their behavior.
- The cached overloads reuse each point mass scalar within the update instead
  of calling `getMass()` repeatedly.

Scene: `soft_bodies`, 200 steps, checkpoint 100, scalar Release build
(`DART_ENABLE_SIMD=OFF`), `COLLISION_DETECTOR=dart`.

| Row | Before parent-term cache | After parent-term cache |
| --- | ---: | ---: |
| Elapsed ms | 39.281 | 38.252 |
| `SoftBodyNode::updateBiasForce` | 9.725 ms | 7.807 ms |
| `SoftBodyNode::updateArtInertia` | 3.588 ms | 2.851 ms |

The exact elapsed row remains host-load-sensitive, but the focused profile rows
show the intended scalar/cache win. This is still not the full data-layout
answer: `SoftBodyNode` remains `std::vector<PointMass*>`, connected-neighbor
access is still pointer-based, and SIMD should wait for contiguous phase data
or a retained SoA facade.

## Contiguous state/property edge-loop slice

The next WP-DB.06 implementation keeps the same public API and point-mass
ownership model, but moves the hottest connected-neighbor bias-force loop one
step closer to cache-friendly data access:

- `SoftBodyNode::updateBiasForce()` now passes the owning soft body's
  contiguous `mPointStates` and `mPointProps` vectors into an internal
  `PointMass::updateBiasForceFD()` overload.
- The connected-neighbor spring loop reads neighbor state directly from the
  `mPointStates` vector using stored connection indices instead of calling
  `getConnectedPointMass(i)->getState()` for every edge contribution.
- `SoftBodyNode::updateBiasForce()` validates the shared point-mass velocity,
  partial-acceleration, and articulated-inertia caches once per soft body before
  entering the point-mass loop. The vector-backed internal overload then reads
  the already computed `PointMass` velocity, partial-acceleration, and implicit
  inertia cache members directly.
- The legacy protected overload remains and performs the same cache validation
  before delegating, so existing internal call sites keep their lazy-update
  behavior.

Scene: `soft_bodies`, 200 steps, checkpoint 100, scalar Release build
(`DART_ENABLE_SIMD=OFF`), `COLLISION_DETECTOR=dart`.

| Row | After parent-term cache reference | Post-lint vector-backed edge loop repeat |
| --- | ---: | ---: |
| Elapsed ms | 38.252 | 48.601 |
| `SoftBodyNode::updateBiasForce` | 7.807 ms | 7.835 ms |
| `SoftBodyNode::updateArtInertia` | 2.851 ms | 3.695 ms |

The step-200 checksum row matched the earlier native `soft_bodies` diagnostic:
`skelPosL1=1.3242580728329107`, `pointPosL1=0.12844450735027887`,
`pointVelL1=6.748189952654168`, and
`pointWorldPosL1=189.54545901295526`. The absolute elapsed row and focused
profile rows remain host-load-sensitive; local repeats during this slice ranged
from 6.695 ms to 11.54 ms for `SoftBodyNode::updateBiasForce`. Treat this as
behavior-preserving data-access evidence, not a stable speedup threshold.

This still does not solve the storage problem completely. Point masses are
still heap-owned objects behind `std::vector<PointMass*>`, and the vectors are
still array-of-struct data rather than SIMD-ready structure-of-arrays. The next
layout step should introduce a retained internal facade/span for phase inputs
before moving formulas into `dart/simd/`.

## Contiguous kinematics cache slice

The next WP-DB.06 implementation applies the same access pattern to the
point-mass kinematics preflight used by the bias-force path:

- `PointMass::updateTransform()` now has an internal overload that receives the
  point state, point properties, and parent world transform directly.
- `PointMass::updateVelocity()` now has an internal overload that receives the
  point state and parent spatial velocity directly. `SoftBodyNode` validates
  the shared transform cache once before the point loop.
- `PointMass::updatePartialAcceleration()` now has an internal overload that
  receives the point state and parent angular velocity directly.
- `SoftBodyNode::{updateTransform,updateVelocity,updatePartialAcceleration}()`
  pass contiguous point state/property entries and cached parent terms through
  those overloads, while the legacy protected `PointMass` methods keep their
  lazy cache checks.

A native `soft_bodies` 200-step single-thread smoke after this slice preserved
the same step-200 checksum row:
`skelPosL1=1.3242580728329107`, `pointPosL1=0.12844450735027887`,
`pointVelL1=6.748189952654168`, and
`pointWorldPosL1=189.54545901295526`. The elapsed/profile rows from this host
were too noisy to claim a stable speedup. Treat this as another
behavior-preserving data-access cleanup on the path toward an internal
phase-input facade.

## Rest-detection memory-hardening carryover

After refreshing `origin/dart6-memory-hardening`, this branch ports its narrow
`Skeleton::checkExternalDisturbanceAndReset()` body-wrench scan:

- The rest-detection query now scans `BodyNode::getExternalForceLocal()`
  directly instead of materializing `Skeleton::getExternalForces()` when the
  external-force projection cache is dirty.
- `_resetCommand` still observes body wrenches that project to zero generalized
  force.
- Generalized force and command scans remain per-DOF and allocation-free.

This is a shared simulation allocation-hardening slice, not the final soft-body
storage answer. It removes an avoidable projection-cache materialization from a
per-step quiet/rest query while keeping the existing DART 6 public API.
`test_SoftDynamics` and `test_DARTCollisionDetector` passed after the carryover.

## `origin/dart6-memory-hardening`

Live remote check on 2026-07-05, refreshed after merging the latest
`origin/release-6.20` into `js/dart6-deformable-performance`:

- tip: `60356be6101` (`Preserve collision result during simulation
  preparation`),
- merge-base with current `origin/release-6.20`: `fd6919a9893`,
- current `origin/release-6.20` tip: `fd6919a9893`.

Relevant lessons from that branch:

- It ports `FrameAllocator` and extends `MemoryManager` toward frame scratch.
- It adds `World`-owned memory management and step-allocation gates.
- Its own task explicitly excludes a hard zero-allocation gate for the
  soft-body path, allowing only cheap pooling improvements.
- Its timing evidence proves zero allocations on native rigid paths but warns
  against broad speedup claims; some Google Benchmark rows regressed while the
  long-running `boxes_headless` loop improved.

Decision for this deformable-body branch:

- Do not merge `origin/dart6-memory-hardening` wholesale into
  `js/dart6-deformable-performance` while this branch is carrying the native
  deformable-collision slices.
- Keep the small direct body-wrench scan ported here because it is shared,
  release-current, and independently tested.
- Treat it as a required coordination dependency for future allocator/layout
  work.
- If that branch lands or is rebased onto current `release-6.20`, soft-body
  layout work should build on its `World`/`MemoryManager`/`FrameAllocator`
  surfaces instead of adding an unrelated soft-body allocator path.
- Before any `PointMass` storage migration, add an allocation-counting soft-body
  gate modeled on that branch's `test_StepAllocation.cpp`, then use
  `soft_body_headless` checksums to preserve behavior.

## Candidate implementation sequence

1. Add an internal soft-body storage facade that can expose contiguous spans of
   point positions, velocities, accelerations, forces, masses, and connectivity
   without changing public `PointMass*` accessors.
2. Keep existing `PointMass` objects initially, but mirror the hottest phase
   inputs into retained scratch so `updateBiasForce` and `updateArtInertia` can
   be measured as contiguous scalar loops.
3. Once scalar checksums match, evaluate replacing per-point allocations with a
   contiguous object pool or memory-manager-backed block allocation.
4. Add SIMD kernels only for phases whose memory layout is contiguous and whose
   FP contract is approved.
5. Add strict allocation gates for native soft-body scenes once the
   memory-hardening branch is available on the current release base.
