# RESUME - DART 6 deformable body feature and performance

Latest state:

- Task folder created on `release-6.20`.
- CPU performance scope is explicit: single-core, multi-core, and SIMD lanes.
- GPU is documented as a DART 6 non-goal unless release-branch evidence changes.
- First code packet adds a headless benchmark target for existing soft-body
  scenes.
- `01-baseline-evidence.md` records the first smoke rows. They prove target
  coverage but are not clean regression thresholds because CPU scaling and host
  load were noisy.
- `02-paper-parity-matrix.md` maps the requested papers into tracked DART 6
  feature, demo, and performance targets.
- `03-stability-gate.md` records the first active `test_SoftDynamics` finite
  state gate across representative SKEL soft scenes and thread settings.
- `04-data-layout-and-memory-hardening.md` records the soft-body data-layout
  risk, adds the `soft_body_headless` profile/checksum runner, and documents why
  `origin/dart6-memory-hardening` is a pattern/dependency source rather than a
  clean branch merge right now.
- First WP-DB.06 optimization is implemented locally: FCL soft meshes now use
  shared vertex topology, preallocate FCL's previous-vertex update buffer at
  geometry creation, and skip local BVH refits when point-mass local positions
  have not changed. `soft_bodies` checksums stayed stable in both `THREADS=1`
  and `THREADS=4` profile runs.
- `soft_body_headless` accepts `COLLISION_DETECTOR=<factory-name>` so future
  runs can compare current FCL behavior against `COLLISION_DETECTOR=dart`.
- First WP-DB.08 native collision slice is implemented locally:
  `SoftMeshShape` now refreshes dynamic bounds for the native detector, and
  native dispatch handles point-mass contacts for soft-vs-plane and soft-vs-box
  pairs in both argument orders.
- Second WP-DB.08 native collision slice is implemented locally: native dispatch
  now handles point-mass contacts for soft-vs-sphere and
  soft-vs-sphere-like-ellipsoid pairs in both argument orders, with focused
  unit coverage.
- Third WP-DB.08 native collision slice is implemented locally: native dispatch
  now handles a soft-vs-soft vertex-face contact lane, populating both soft
  triangle IDs for `SoftContactConstraint`. The broader `soft_bodies` native
  diagnostic no longer emits unsupported shape-pair errors.
- Fourth WP-DB.08 native collision slice is implemented locally:
  `DARTCollisionObject` now keeps a backend-internal soft mesh collision cache
  with local vertices, point-to-first-face metadata, and precomputed triangle
  geometry. The soft-soft lane uses this cache and local-frame point projection
  instead of recomputing face geometry through `PointMass*` lookups for every
  point-face pair.
- A follow-up WP-DB.08 soft-soft optimization slice is implemented locally:
  cached soft faces now retain their plane offsets, and the vertex-face lane
  rejects non-contact-side face candidates before running the AABB and
  barycentric projection path.
- Another WP-DB.08 soft-soft optimization slice is implemented locally:
  `DARTCollisionObject` now keeps retained soft-face BVH nodes and face-index
  ordering. The native soft-soft query keeps the old linear path until a
  penetrating candidate exists, then traverses the retained BVH and prunes face
  groups whose AABB lower bound cannot beat the current candidate.
- Fifth WP-DB.08 native collision slice is implemented locally: native dispatch
  now handles soft-vs-non-spherical-ellipsoid point contacts in both argument
  orders. Non-spherical ellipsoid primitive pairs are no longer silently treated
  as spheres; they fall through to the existing unsupported-pair diagnostic
  unless the ellipsoid is sphere-like.
- `05-native-collision-deformable-lane.md` records the focused native tests and
  a 200-step `drop_box` FCL/native parity run. The checksums matched exactly,
  and native was faster on that limited soft-box scene on this host. The latest
  validation run measured about 2.75x, but the exact timing ratio is
  host-load-sensitive evidence rather than a stable threshold.
- A broader diagnostic `COLLISION_DETECTOR=dart` `soft_bodies` run now completes
  without shape-creation or unsupported-pair warnings for the representative
  soft scene. After the early separation-rejection slice, the latest 20-step
  diagnostic measured `DARTCollide soft-soft vertex-face` at 963.26 us over 60
  calls on this host, down from the earlier 9.138 ms scan-only result. Treat
  the exact timing as host-load-sensitive smoke evidence; the remaining hotspot
  still needed a real retained face BVH or active-neighborhood path before this
  could be treated as representative native performance.
- After the retained soft-face BVH slice, a 200-step same-scene native run
  measured 39.281 ms elapsed with `THREADS=1`, while the same-scene FCL row
  measured 297.851 ms. The native and FCL `soft_bodies` checksums still differ
  because native soft-soft remains a vertex-face approximation, but native
  checksums matched between `THREADS=1` and `THREADS=4`. `THREADS=4` was slower
  for both backends on this small scene, so finite-finite soft-soft
  parallelization remains an open multi-core scaling gap.
- A WP-DB.06 scalar/cache slice is implemented locally: `SoftBodyNode` now
  computes parent angular velocity, local gravity, stiffness, and damping terms
  once per point-mass dynamics phase and passes them through protected
  cached-parameter `PointMass` overloads. The latest native `soft_bodies`
  200-step row measured 38.252 ms elapsed; `SoftBodyNode::updateBiasForce`
  measured 7.807 ms and `SoftBodyNode::updateArtInertia` measured 2.851 ms.
  This is a scalar/cache improvement, not the final contiguous storage or SIMD
  solution.
- A follow-up WP-DB.06 point-state/property slice is implemented locally:
  `SoftBodyNode::updateBiasForce()` passes contiguous point states and
  properties into an internal vector-backed `PointMass::updateBiasForceFD()`
  overload, validates shared velocity, partial-acceleration, and
  articulated-inertia caches once per soft body, and the connected-neighbor
  spring loop reads neighbor state by connection index instead of through
  `PointMass*` lookups. A native
  `soft_bodies` 200-step profile repeat preserved the prior checksum and
  measured `SoftBodyNode::updateBiasForce` at 7.835 ms after lint on this host,
  with local repeats ranging from 6.695 ms to 11.54 ms. Treat the exact elapsed
  rows as host-load-sensitive evidence, not a stable threshold.
- Another WP-DB.06 kinematics cache slice is implemented locally:
  `SoftBodyNode::{updateTransform,updateVelocity,updatePartialAcceleration}()`
  pass contiguous point state/property entries and cached parent
  transform/velocity terms into protected `PointMass` overloads. A native
  `soft_bodies` 200-step smoke preserved the prior checksum; timing remained
  too noisy for a stable speedup claim.
- A narrow `origin/dart6-memory-hardening` carryover is implemented locally:
  `Skeleton::checkExternalDisturbanceAndReset()` now scans body-local external
  wrenches directly instead of materializing the external-force projection cache
  during rest detection. This is shared allocation/per-step hardening, not a
  replacement for soft-body contiguous storage.
- A WP-DB.07/WP-DB.08 pair-level multicore slice is implemented locally:
  finite-finite native soft-soft candidate pairs can run in worker-local
  `CollisionResult`s and then merge in the original sweep order. The
  `soft_bodies` step-200 checksum matched exactly between `THREADS=1` and
  `THREADS=4`; local timing was noisy, so treat the evidence as worker
  activation/correctness rather than a stable scaling threshold.
- A follow-up WP-DB.08 native cache slice is implemented locally:
  soft-vs-primitive contact loops now reuse `DARTCollisionObject` cached local
  vertices and first-face metadata instead of scanning faces per point. The
  `drop_box` 200-step FCL/native checksum parity smoke still matched exactly
  after this change.
- Branch `js/dart6-deformable-performance` is based on `release-6.20` commit
  `d0d8fa1a495`; local `origin/release-6.20` has advanced, so merge the latest
  target branch before any PR push.

Next steps:

1. Re-run the benchmark on an idle host with `--benchmark_min_time=1s` and
   repetitions, then refresh `01-baseline-evidence.md`.
2. Use `soft_body_headless` for longer single-core and multi-core profile
   captures on an idle host, then choose the next measured soft-body layout
   slice. The likely next WP-DB.06 slice is a retained internal span/facade for
   point-mass phase inputs before any `dart/simd/` kernel.
3. Extend `test_SoftDynamics` beyond finite-state checks with energy or state
   hash regression thresholds that are stable across supported platforms.
4. Complete WP-DB.04 soft-body matrix/vector aggregation, then re-enable the
   equations-of-motion checks that remain disabled in `test_SoftDynamics.cpp`.
5. Complete the paper parity matrix with representative scenes and numbers from
   Kim/Pollard 2011 and Jain/Liu 2011.
6. Continue WP-DB.08 with fuller triangle/contact-neighborhood coverage and
   stronger multicore scaling beyond the current pair-level soft-soft worker
   path before preferring native as the default soft collision backend.
7. Add allocation gates for native soft-body scenes once the
   `dart6-memory-hardening` surfaces are available on the current release base.
8. Before creating the PR, capture a same-host baseline-vs-branch performance
   comparison and include newly added GUI example commands plus locally captured
   videos as PR evidence.

Do not mark this task complete until every work packet in `README.md` has
current evidence or a maintainer-approved deferral in a durable owner doc.
