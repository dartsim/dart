# WP-DB.08 native collision deformable lane

Captured on 2026-07-05 from local branch
`js/dart6-deformable-performance`.

## Direction

Native collision should be the preferred DART collision solution for DART-owned
runtime paths, including deformable-body work. FCL remains necessary in this
branch for broad soft-mesh coverage, but it is no longer the only measured path:
this branch now has an initial native `SoftMeshShape` lane for soft-vs-plane,
soft-vs-box, soft-vs-sphere, soft-vs-ellipsoid, and soft-vs-soft contacts. The
release-branch target remains to move soft/deformable collision onto
`dart/collision/native` once complete parity and performance gates exist.

## Current native evidence

- `dart/collision/native/shapes/shape.hpp` has `native::MeshShape` with
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
  cache is unavailable.
- `tests/unit/collision/test_DARTCollisionDetector.cpp` now covers native
  soft-plane, soft-box, soft-sphere, soft-sphere-like-ellipsoid,
  soft-non-spherical-ellipsoid, and soft-soft vertex-face contacts with
  expected object ordering, normals, penetration, and valid soft triangle IDs.

## Harness support

`soft_body_headless` now accepts:

```bash
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
2. Add broader soft-vs-primitive coverage beyond the current representative
   soft scene.
3. Decide whether the next native soft path should use full triangle-mesh
   collision, adaptive active-vertex contact neighborhoods, or both.
4. Extend the retained topology/dynamic-vertex adapter toward active
   neighborhoods and allocation gates for soft-body scenes.
5. Add allocation gates for native soft-body scenes once the
   `dart6-memory-hardening` surfaces are available on the current release base.
6. Extend parity tests to compare contact counts, representative contact points,
   checksums, and timing between FCL and native on the same soft scenes.
7. Once parity is green, make the native path the preferred deformable collision
   backend and keep FCL as compatibility/fallback coverage.

## Acceptance gate

Native soft/deformable collision should not replace FCL until:

- `test_SoftDynamics` passes with the native detector on representative soft
  scenes,
- `soft_body_headless` reports stable checksums for `COLLISION_DETECTOR=dart`,
- native detector rows match or beat FCL on the representative
  `soft_bodies`, `soft_open_chain`, and contact-heavy soft scenes on the same
  host,
- unsupported-pair diagnostics are eliminated for the representative soft
  scene set,
- downstream collision/constraint compatibility gates are run because this
  surface can affect Gazebo and gz-physics.
