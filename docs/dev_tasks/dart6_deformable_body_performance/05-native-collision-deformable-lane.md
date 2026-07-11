# WP-DB.08 native collision deformable lane

Captured on 2026-07-05 from local branch
`js/dart6-deformable-performance`.

## Direction

Native collision should be the preferred DART collision solution for DART-owned
runtime paths, including deformable-body work. FCL remains necessary in this
branch for broad soft-mesh coverage, but it is no longer the only measured path:
this branch now has an initial native `SoftMeshShape` lane for soft-vs-plane,
soft-vs-box, soft-vs-sphere, soft-vs-ellipsoid, and soft-vs-soft contacts. The
direct `NativeCollisionDetector` factory key also participates in these soft
scenes through cached DART-native fallback collision objects instead of
skipping `SoftMeshShape` or `EllipsoidShape`. The release-branch target remains
to move soft/deformable collision onto `dart/collision/native` once complete
parity and performance gates exist.

## Current native evidence

- `dart/collision/native/shapes/Shape.hpp` has `native::MeshShape` with
  retained vertices, triangle indices, and a local triangle BVH.
- `dart/collision/native` already owns primitive narrow-phase pieces such as
  sphere-sphere, box-box, GJK/MPR-style support-mapped collision, and batch
  helpers.
- `DARTCollisionDetector` has a thread pool and finite/plane broadphase paths,
  with `DARTCollisionDetector::setNumCollisionThreads()` wired through
  `ConstraintSolver::setNumSimulationThreads()`.
- `DARTCollisionObject::CachedShapeKind` now caches `SoftMeshShape` and forces
  `SoftMeshShape::refreshData()` on every native update so dynamic soft bounds
  track point-mass motion even when the shape frame version is unchanged.
- Direct `NativeCollisionObject` instances keep cached DART-native fallback
  objects for `SoftMeshShape` and `EllipsoidShape` pairs. Their dynamic AABBs
  stay in the native broadphase, fallback plane pairs use the cached DART plane
  collision path, and fallback pair scratch results skip unused colliding-object
  lookup-cache maintenance.
- `BruteForceBroadPhase` now stores deterministic object IDs and AABBs in a
  sorted contiguous entry vector, with an ID-to-index map used only for
  add/update/remove. The hot pair walk no longer performs hash lookups per AABB
  candidate.
- `DARTCollisionObject` also keeps a backend-internal soft mesh cache with
  current local vertices, first-face metadata per point mass, and precomputed
  triangle geometry for native soft-soft contact tests. The cache resizes when
  topology changes and reuses retained storage during steady-state collision
  updates. Cached soft faces now also retain their plane offset so the
  soft-soft lane can reject separated face candidates before AABB and
  barycentric projection work. The cache now also retains a soft-face BVH and
  face-index ordering so penetrating vertex-face queries can prune face groups
  without rebuilding topology storage.
- `SoftMeshShape::updateBoundingBox()` now computes bounds from current
  point-mass local positions instead of leaving a stale TODO path.
- `DARTCollide.cpp` now dispatches `SoftMeshShape` against `PlaneShape`,
  `BoxShape`, `SphereShape`, `EllipsoidShape`, and `SoftMeshShape` in both
  argument orders. This first lane generates contacts from penetrating point
  masses and assigns a representative soft face ID for the contact triangle
  field. The soft-vs-primitive loops now reuse cached local vertices and
  first-face metadata, falling back to the old face scan only when the retained
  cache is unavailable. A follow-up primitive-frame slice classifies
  soft-vs-plane, soft-vs-sphere, and soft-vs-box points in primitive-local
  coordinates and computes world contact points only for colliding vertices.
- `tests/unit/collision/test_DARTCollisionDetector.cpp` now covers native
  soft-plane, soft-box, soft-sphere, soft-sphere-like-ellipsoid,
  soft-non-spherical-ellipsoid, and soft-soft vertex-face contacts with
  expected object ordering, normals, penetration, and valid soft triangle IDs.
- `tests/integration/test_SoftDynamics.cpp` now runs the representative
  finite-state and one-thread versus four-thread final-state gate under both
  the default detector and `CollisionDetectorType::Dart`.
- `tests/integration/test_StepAllocation.cpp` now has native soft-box
  allocation gates for explicit first post-bake and implicit second-step
  simulation. The current local result is zero `operator new`, zero raw
  `malloc`, and zero counted base allocator growth on the measured soft contact
  steps.

## Harness support

`soft_body_headless` now accepts:

```bash
COLLISION_DETECTOR=native ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless soft_bodies 200 100
COLLISION_DETECTOR=dart ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless soft_bodies 200 100
COLLISION_DETECTOR=fcl ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless soft_bodies 200 100
```

The default remains DART's existing `ConstraintSolver` default collision
detector. The detector selector is intended for parity and regression evidence;
it is not a claim that native soft collision is complete today.

## First native implementation slice

The first WP-DB.08 implementation supports:

- dynamic `SoftMeshShape` bounding boxes in the native collision cache,
- native point-mass contact against planes,
- native point-mass contact against boxes,
- native point-mass contact against spheres,
- native point-mass contact against sphere-like and non-spherical ellipsoids,
- native soft-soft vertex-face contact, with one best face selected per point
  mass and both soft triangle IDs populated for `SoftContactConstraint`,
- retained native soft mesh collision cache data for the soft-soft lane,
- retained native soft mesh collision cache data for soft-vs-primitive local
  vertices and representative soft face IDs,
- early contact-side separation rejection before soft-soft projected-triangle
  tests,
- retained soft-face BVH traversal for the soft-soft lane once a penetrating
  candidate exists, with a linear prepass fallback for no-contact and
  non-penetrating shell cases,
- finite-finite native soft pair worker scheduling that collides eligible
  soft-soft candidate pairs in worker-local `CollisionResult`s and merges them
  in the original sweep order to preserve duplicate-contact handling and
  contact-cap behavior,
- primitive-frame classification for soft-vs-plane, soft-vs-sphere, and
  soft-vs-box point contacts so non-contact vertices avoid unnecessary world
  contact-point transforms,
- direct `NativeCollisionDetector` soft/ellipsoid fallback that keeps native
  broadphase culling while reusing the tested DART-native soft contact kernels,
- cache-friendly contiguous storage for native brute-force broadphase entries,
- reusable soft contact constraints with fixed one-contact, Jacobian, and
  tangent-basis storage for the current soft-contact constraint shape,
- detector selection in `soft_body_headless` for FCL/native comparison.

The contact lane is deliberately limited. It detects point masses that penetrate
the primitive and does not yet detect all triangle-face intersections when no
point mass enters the primitive. The soft-soft lane is likewise a vertex-face
shell, not a full triangle-triangle mesh collider. The latest slice replaces
the pure all-faces scan with a hybrid path: it keeps the old linear scan when no
penetrating candidate exists, then traverses a retained face BVH and prunes
nodes whose AABB lower bound cannot beat the current penetrating candidate.
This preserves the current best-face semantics while reducing contact-heavy
soft-soft work.

Focused native tests:

```bash
cmake --build build/default/cpp/Release --target test_DARTCollisionDetector --parallel
ctest --test-dir build/default/cpp/Release -R 'test_DARTCollisionDetector$' --output-on-failure
```

Both commands passed locally on 2026-07-05 after this slice.

Headless parity smoke on a soft-box scene:

```bash
COLLISION_DETECTOR=fcl ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless drop_box 200 100
COLLISION_DETECTOR=dart ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless drop_box 200 100
```

Both detectors produced the same step-200 checksums:

| Checksum | Value |
| --- | ---: |
| `skelPosL1` | `0.33013838249780086` |
| `skelPosSq` | `0.038880346761000072` |
| `skelVelL1` | `1.9620000000000022` |
| `skelVelSq` | `3.849444000000005` |
| `pointPosL1` | `5.6153425542514667e-17` |
| `pointPosSq` | `6.2626350447868326e-35` |
| `pointVelL1` | `4.4369015125248549e-16` |
| `pointVelSq` | `3.9372190063690669e-33` |
| `pointWorldPosL1` | `8.3885820947800589` |
| `pointWorldPosSq` | `1.3492978157860007` |

Timing from the same local run:

| Detector | Elapsed ms | Steps/s | Notes |
| --- | ---: | ---: | --- |
| FCL | 19.526 | 10243.0 | `FCLCollisionObject::updateSoftMesh refit` took 12.75 ms over 199 calls and dominated the profile. |
| Native | 7.107 | 28142.9 | `collide` took 962.34 us over 200 calls; `SoftMeshShape::update` took 42.95 us over 200 calls. |

This is about a 2.75x elapsed-time speedup for this limited soft-box scene on
this host in the latest validation run. Treat the exact ratio as
host-load-sensitive evidence, not a stable threshold. It is valid evidence that
native can beat FCL for the implemented soft-box contact lane, not evidence that
native is complete for all soft-body collision.

After routing soft-vs-primitive loops through the retained soft point cache, the
same `drop_box` 200-step FCL/native parity smoke still produced identical
checksums. A local repeat measured 20.159 ms for FCL and 7.473 ms for native;
the native row remained dominated by dynamics and object update rather than
per-point face lookup.

After stacking on `origin/dart6-memory-hardening` and adding the soft contact
allocation gate, a current 200-step repeat again produced identical FCL/native
`drop_box` checksums. The same busy-host run measured 48.534 ms for FCL and
11.977 ms for native. Treat that as current parity and native-path dominance
evidence, not a stable speedup threshold.

After the primitive-frame soft-vs-primitive slice, a 2026-07-07 repeat again
produced identical FCL/native `drop_box` step-200 checksums. On the same busy
host, FCL measured 7.045 ms elapsed and native measured 1.431 ms elapsed for
200 steps. A quick current-only Google Benchmark smoke still ranked native
ahead of FCL on every tracked scene/thread row:

| Scene | Threads | Native CPU ms | FCL CPU ms |
| --- | ---: | ---: | ---: |
| `adaptive_deformable` | 1 | 2.01 | 16.4 |
| `adaptive_deformable` | 16 | 2.41 | 17.0 |
| `soft_cubes` | 1 | 3.69 | 17.8 |
| `soft_cubes` | 16 | 3.68 | 17.9 |
| `soft_bodies` | 1 | 13.8 | 68.1 |
| `soft_bodies` | 16 | 13.6 | 63.4 |
| `soft_open_chain` | 1 | 5.38 | 29.6 |
| `soft_open_chain` | 16 | 5.06 | 29.8 |

The load average during that benchmark was still high, so these rows are
current smoke evidence for detector ranking and regression triage, not final
threshold-quality PR evidence.

Diagnostic run on a broader soft scene:

```bash
COLLISION_DETECTOR=dart ./build/default/cpp/Release/tests/benchmark/integration/soft_body_headless soft_bodies 20 20
```

After the soft-soft vertex-face slice, the run completed and printed
`detector=dart` with no unsupported shape-pair diagnostics. After the
soft-non-spherical-ellipsoid slice, the same diagnostic also stopped emitting
the earlier `EllipsoidShape` shape-creation warning. The diagnostic now proves
that native pair dispatch reaches the broader representative soft scene without
reported native coverage holes. Longer same-host FCL/native runs are still
needed before treating the timing as representative full-scene performance
evidence.

The earlier 20-step diagnostic exposed the initial native soft-soft bottleneck:

| Scope | Total | Calls | Share |
| --- | ---: | ---: | ---: |
| `DARTCollide soft-soft vertex-face` | 9.138 ms | 60 | 85.1% |

After adding the retained `DARTCollisionObject` soft mesh cache and local-frame
point projection, the post-lint repeat of the same command produced the same
step-20 checksum row and reduced the soft-soft scope:

| Scope | Total | Calls | Share |
| --- | ---: | ---: | ---: |
| `DARTCollide soft-soft vertex-face` | 3.170 ms | 60 | 46.5% |

After the soft-non-spherical-ellipsoid slice, a repeat of the same command
completed without shape-creation or unsupported-pair warnings and measured:

| Scope | Total | Calls | Share |
| --- | ---: | ---: | ---: |
| `DARTCollide soft-soft vertex-face` | 1.364 ms | 60 | 43.6% |

After caching face plane offsets and moving the contact-side separation check
ahead of the projected-triangle path, the same command preserved the step-20
checksum row and measured:

| Scope | Total | Calls | Share |
| --- | ---: | ---: | ---: |
| `DARTCollide soft-soft vertex-face` | 963.26 us | 60 | 25.2% |

After adding retained soft-face BVH traversal with a linear no-contact prepass,
a repeat of the 20-step diagnostic preserved the step-20 checksum row and
measured:

| Scope | Total | Calls | Share |
| --- | ---: | ---: | ---: |
| `DARTCollide soft-soft vertex-face` | 1.015 ms | 60 | 34.4% |

That short early-scene diagnostic is mostly a coverage smoke; it is not the
case where BVH pruning pays for itself. Before the finite-finite worker slice,
a 200-step same-scene run exercised the later contact-heavy span and gave a
clearer signal:

| Detector | Threads | Elapsed ms | Steps/s | Native/FCL profile note |
| --- | ---: | ---: | ---: | --- |
| FCL | 1 | 111.695 | 1790.6 | `FCLCollisionObject::updateSoftMesh refit` took 80.37 ms over 480 calls. |
| Native | 1 | 38.252 | 5228.5 | `DARTCollide soft-soft vertex-face` took 12.29 ms over 600 calls. |
| FCL | 4 | 116.931 | 1710.4 | Same FCL checksum row as `THREADS=1`; refit remained dominant. |
| Native | 4 | 38.211 | 5234.1 | Same native checksum row as `THREADS=1`; finite-finite soft-soft collision remains serial. |

After adding finite-finite soft pair worker scheduling, the 200-step native
`soft_bodies` checksum row still matched exactly between `THREADS=1` and
`THREADS=4`. A same-host local repeat that was not run as a stable benchmark
measured 44.199 ms for `THREADS=1` and 41.988 ms for `THREADS=4`; later
follow-up runs on the same busy host varied substantially but continued to show
the worker scope (`DART native finite-finite soft workers`) and identical
checksums. Treat this as correctness and worker-activation evidence, not as a
settled scaling number. The scene only has a few soft-soft finite-finite pairs,
so larger multicore gains likely require intra-pair point/face parallelism,
larger representative scenes, or active-neighborhood contact work.

The native and FCL step-200 checksum rows are not identical on the broader
`soft_bodies` scene because the native soft-soft lane is still a different
vertex-face contact approximation. The current evidence therefore supports
"native is much faster for the implemented lane and stable across thread
settings," not full FCL parity. The latest rows also include the
backend-independent parent-term cache slice from
`04-data-layout-and-memory-hardening.md`, which reduced the point-mass dynamics
profile rows but does not change the remaining native soft-soft coverage gap.

The cache-backed lane is still not the destination implementation for dense or
contact-heavy soft scenes. It removes repeated `PointMass*` lookups and
triangle setup from the inner loop, rejects separated faces before projection,
now prunes retained BVH nodes after a penetrating candidate exists, and can run
eligible soft-soft finite pairs through the collision thread pool. The next
native soft-soft packet should still add full triangle/contact-neighborhood
coverage and stronger multicore scaling before native can be the default
deformable backend.

## Remaining native work

1. Optimize native soft-soft collision beyond the current hybrid retained-BVH
   vertex-face lane, including intra-pair or active-neighborhood parallelism for
   scenes with only a few finite-finite soft pairs.
2. Replace the direct `NativeCollisionDetector` soft/ellipsoid fallback bridge
   with native-owned kernels once full triangle/contact-neighborhood coverage is
   ready.
3. Decide whether the next native soft path should use full triangle-mesh
   collision, adaptive active-vertex contact neighborhoods, or both.
4. Extend the retained topology/dynamic-vertex adapter toward active
   neighborhoods for soft-body scenes.
5. Extend parity tests to compare contact counts, representative contact points,
   checksums, and timing between FCL and native on the same soft scenes.
6. Once parity is green, make the native path the preferred deformable collision
   backend and keep FCL as compatibility/fallback coverage.

## Acceptance gate

Native soft/deformable collision should not replace FCL until:

- `test_SoftDynamics` passes with the native detector on representative soft
  scenes. This is now covered by the finite-state and one-thread versus
  four-thread final-state gate; broader physical regression thresholds remain
  useful follow-up coverage,
- `soft_body_headless` reports stable checksums for `COLLISION_DETECTOR=dart`
  and direct `COLLISION_DETECTOR=native`,
- direct `native` rows match or beat FCL and any other checksum-equivalent
  backend on representative `soft_bodies`, `soft_open_chain`, and contact-heavy
  soft scenes on the same host,
- native soft allocation gates cover the representative soft scene set after
  simulation preparation, including contact-heavy SKEL windows,
- unsupported-pair diagnostics are eliminated for the representative soft
  scene set,
- downstream collision/constraint compatibility gates are run because this
  surface can affect Gazebo and gz-physics.

## Long-run determinism rows (2026-07-11)

2000-step `soft_body_headless` checksum rows at commit `578ea17a049` across
{drop_box, soft_cubes, soft_bodies, soft_open_chain, adaptive_deformable} x
{dart, native}:

- **Thread invariance**: `THREADS=1` vs `THREADS=16` step-2000 checksums are
  bit-identical for all 10 scene/detector combinations.
- **Rerun repeatability**: repeated single-thread 2000-step runs are
  bit-identical per backend (spot-checked on the two scenes below).
- **Cross-backend**: `dart` and direct `native` step-2000 checksums are
  bit-identical on `soft_cubes`, `soft_open_chain`, and
  `adaptive_deformable`; `drop_box` and `soft_bodies` diverge by step 2000
  even though both matched bit-exactly at step 200. The two backends share
  the narrow-phase kernels but not every FP evaluation path, so ULP-scale
  differences amplify chaotically over long contact-rich runs. The codified
  equivalence bar remains the 200-step tolerance-based correctness gate in
  the comparison harness; per-backend determinism (thread invariance and
  rerun repeatability) is the long-run acceptance component and holds
  everywhere.
