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
- `03-stability-gate.md` records the active `test_SoftDynamics` finite-state
  gate across representative SKEL soft scenes and thread settings, plus the
  ordered final-state comparison between `threads=1` and `threads=4`.
- `07-equation-correctness.md` records the active WP-DB.04 gate:
  point-mass mass now contributes deterministically to
  `Skeleton::getMassMatrix()` and `Skeleton::getAugMassMatrix()`, soft-body
  trees return inverse mass and inverse augmented mass matrices consistent with
  those public matrices, and point-mass gravity contributes to
  `Skeleton::getGravityForces()` and
  `Skeleton::getCoriolisAndGravityForces()` at rest. Point-mass external
  forces are projected through the parent soft body into
  `Skeleton::getExternalForces()` and cleared by
  `Skeleton::clearExternalForces()`. The test compares the observed deltas
  against analytical point-mass Jacobian and gravity-wrench projections and
  verifies both left and right inverse-matrix identity products before and
  after a point-mass mass change. The disabled random soft-body
  `compareEquationsOfMotion` body has been replaced with
  `representativeEquationMatrixAndVectorChecks`, covering a rigid-only world,
  a soft-body world, and a mixed rigid/soft world with deterministic
  matrix/vector equation checks.
- `04-data-layout-and-memory-hardening.md` records the soft-body data-layout
  risk, adds the `soft_body_headless` profile/checksum runner, and records that
  this branch now stacks on `origin/dart6-memory-hardening` for
  `World`/`MemoryManager`/`FrameAllocator` allocation surfaces.
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
- Another WP-DB.06 scalar data-access slice is implemented locally:
  `SoftBodyNode::{updateBiasImpulse,updateTransmittedImpulse,updateConstrainedTerms}()`
  now update point-mass impulse caches and contiguous point state/property
  vectors directly while preserving the old point-mass velocity and
  acceleration dirty-notice behavior. This removes more per-point helper calls
  from recursive dynamics phases, but it is still not contiguous point-mass
  object storage or SIMD. Native `soft_bodies` 200-step checksum rows matched
  exactly between `THREADS=1` and `THREADS=4` after this slice.
- Another WP-DB.06 internal-facade slice is implemented locally:
  `SoftBodyNode.cpp` now has a local `PointMassPhaseView` for converted
  recursive dynamics phases, centralizing the matched point-object,
  point-state, and point-property access shape without changing public headers
  or class layout. Focused soft dynamics, soft allocation, and native
  `soft_bodies` single-thread/four-thread checksum smokes passed after this
  slice. Retained SoA scratch, contiguous point-mass object storage, and SIMD
  remain open.
- A follow-up WP-DB.06 aggregation slice is implemented locally:
  mass, augmented-mass, inverse-mass helper, gravity, combined-vector, and
  external-force aggregation paths now use the same `PointMassPhaseView`
  instead of delegating through per-point helper methods for behavior-preserving
  cache fills. Focused equation, allocation, and native checksum smokes passed
  after this slice.
- A narrow `origin/dart6-memory-hardening` carryover is implemented locally:
  `Skeleton::checkExternalDisturbanceAndReset()` now scans body-local external
  wrenches directly instead of materializing the external-force projection cache
  during rest detection. This is shared allocation/per-step hardening, not a
  replacement for soft-body contiguous storage. After the full stack was merged
  locally, the conflict resolution also preserved the memory-hardening branch's
  point-mass external-force scan.
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
- The branch is now locally stacked on `origin/dart6-memory-hardening` tip
  `60356be6101`. The merge brings in `FrameAllocator`, `World`-owned memory
  management, and the allocation-counting integration-test harness used by the
  soft-body gate.
- A native soft-body allocation gate is implemented locally:
  `StepAllocation.*Soft*` builds a 3x3x3 soft box over ground with
  `CollisionDetectorType::Dart` and verifies explicit first post-bake and
  implicit second-step contact solves after warmup. Current focused output has
  9 contacts and zero `operator new`, zero raw `malloc`, and zero counted base
  allocator growth for all four soft allocation rows.
- The soft allocation gate is backed by fixed-size `SoftContactConstraint`
  storage, reusable soft contact constraints in `ConstraintSolver`, and
  retained `World::updateRestStates()` scratch vectors for initial-contact
  skeleton sets. This proves the current native soft-box contact lane can step
  without heap growth after preparation; broader soft scenes still need gates.
- A broader native soft allocation gate is implemented locally: a two-soft-box
  stack over ground warms up, then measures 100 steady-state native steps with
  27 contacts, including 18 soft-soft contacts, and zero `operator new`, zero
  raw `malloc`, and zero counted base allocator growth. This extends coverage
  to a persistent native soft-soft contact lane.
- A representative SKEL allocation gate is implemented locally:
  `dart://sample/skel/softBodies.skel` is parsed, its skeletons are transferred
  into a counted native world, then 100 steady-state steps are measured after
  warmup with zero `operator new`, zero raw `malloc`, and zero counted base
  allocator growth. That measured window has zero contacts, so it proves
  no-contact SKEL soft-dynamics allocation behavior. Follow-up gates now cover
  `dart://sample/skel/soft_open_chain.skel` as another no-contact SKEL
  soft-dynamics window and `dart://sample/skel/soft_cubes.skel` as a
  contact-producing SKEL window with 9 contacts; all four new global/base and
  raw gates report zero `operator new`, zero raw `malloc`, and zero counted
  base allocator growth. The underlying runtime fix reuses all-resting
  kinematic snapshot storage and copies generalized positions by scalar index
  instead of allocating temporary `Eigen::VectorXd` values.
- Current post-merge smokes preserved the key runtime checks: FCL and native
  `drop_box` 200-step checksums matched exactly, native remained faster on that
  limited lane in the local run, and `THREADS=4`
  `COLLISION_DETECTOR=dart soft_bodies 20 20` preserved the prior checksum
  while showing the native finite-finite soft worker scope in the profiler.
- `06-pr-evidence.md` now records a temporary same-harness
  `origin/release-6.20` baseline comparison, current native/FCL headless parity
  evidence, and the fact that this branch has not added a new GUI example or
  local GUI video artifact yet. The native-detector benchmark rows are the
  relevant speedup evidence for WP-DB.08; default-backend CPU rows remain
  noisy/mixed under the current host load.
- Local broad validation after the evidence commit passed `pixi run build`,
  `pixi run test`, and `pixi run test-py`. `pixi run -e gazebo test-gz`
  passed the gz-physics suite and performance checks, then hit a single
  downstream gz-sim `INTEGRATION_entity_system` timing failure; immediate
  focused rerun and fresh `pixi run -e gazebo test-gz-sim` both passed.
- Branch `js/dart6-deformable-performance` now has local implementation commit
  `09b4e0e5d78`, memory-hardening merge/evidence commits
  `3a2833a6e24..9d4ef692eb6`, and a local stack on
  `origin/dart6-memory-hardening`.

Next steps:

1. Re-run the benchmark on an idle host with `--benchmark_min_time=1s` and
   repetitions, then refresh `01-baseline-evidence.md`.
2. Use `soft_body_headless` for longer single-core and multi-core profile
   captures on an idle host, then choose the next measured soft-body layout
   slice. The likely next WP-DB.06 slice is retained SoA scratch for
   point-mass phase inputs before any `dart/simd/` kernel.
3. Extend `test_SoftDynamics` beyond finite-state and one-thread versus
   four-thread final-state checks with energy, contact-force, CoP, or
   historical-golden regression thresholds that are stable across supported
   platforms.
4. Complete the paper parity matrix with representative scenes and numbers from
   Kim/Pollard 2011 and Jain/Liu 2011.
5. Continue WP-DB.08 with fuller triangle/contact-neighborhood coverage and
   stronger multicore scaling beyond the current pair-level soft-soft worker
   path before preferring native as the default soft collision backend.
6. Before creating the PR, capture a same-host baseline-vs-branch performance
   comparison and include newly added GUI example commands plus locally captured
   videos as PR evidence. The first evidence packet exists in
   `06-pr-evidence.md`; rerun it on a cleaner host before turning the smoke
   rows into PR threshold claims.

Do not mark this task complete until every work packet in `README.md` has
current evidence or a maintainer-approved deferral in a durable owner doc.
