# RESUME - DART 6 deformable body feature and performance

## 2026-07-10 overnight state

- A 64-agent adversarial stitch review of merge `4286fd53097` confirmed zero
  semantic merge defects; surviving minor findings were fixed in
  `7d1615b2795` (PointMass::setMass dirty protocol via
  `SoftBodyNode::handlePointMassMassChange()`, dead FCL friend removal, dead
  `findContainingBoxFace()` removal) with a new equation-gate assertion
  pinning the pre-existing rigid-only `Skeleton::getMass()` accounting split
  (recorded in `07-equation-correctness.md`).
- The WP-DB.05 design (`08-adaptive-contact-activation.md`, commit
  `d689aa660f5`) was rewritten after a three-lens design review: frozen
  points are rigid lumps (full articulated inertia + rigid-ride bias +
  boundary-spring reactions), constraint-time seeding, explicit one-step lag
  with rigid first-contact semantics, rest-gated linger with decay to rest
  shape, all-active public matrix semantics, notifier-subclass storage with
  clone/copy transfer, and honestly scoped performance rows.
- WP-DB.06 "retained SoA scratch" completed as a measured reject
  (`c743a451e96` records it in `04-data-layout-and-memory-hardening.md`):
  three mirror variants were all slower than baseline; next step is
  contiguous point-mass object storage with profiler scopes added to the
  soft phases first (the merged `SoftBodyNode.cpp` currently has none).
- The comparison evaluator now implements the documented "matches or beats"
  wording via `--expected-fastest-tie-tolerance` (default 2%,
  `1deef52b8d7`), reported explicitly in `summary.md`.
- A formal current/parent/base matrix run was launched overnight at
  `c743a451e96` with parent `c0fd6742566` (pre-reunification) and base
  `origin/release-6.20`, output dir
  `.benchmark_results/wp-db-reunified-c743a45-parent-c0fd674-base-db255a0`.
  It self-gates on host idleness. Treat its `summary.md` as the reunified
  branch's first full-matrix evidence; the final PR matrix still reruns on
  final history.

## 2026-07-09 evening reunification update

This update corrects the current-reality picture; bullets below it describe
history that partially landed through other refs.

- PR #3307 (merged 2026-07-07) carried an earlier snapshot of this task's
  implementation into `origin/release-6.20`.
- The newer slices (WP-DB.04 equation gates plus `07-equation-correctness.md`,
  the `PointMassPhaseView` series, FCL soft-mesh read slices, World
  empty-solve fast paths, `soft_open_chain`/`soft_cubes` SKEL allocation
  gates) lived only on `wp-db-soft-skel-allocation-gates` and were absent from
  `wp-db-native-soft-fallback` even though this file described them as current
  state.
- Merge commit `4286fd53097` reunifies the lanes on
  `wp-db-native-soft-fallback`. Conflict policy, validation evidence, and the
  accepted 1-3 ULP point-mass accumulator drift on dart/native contact scenes
  are recorded in `decisions.md` (entry 7) and the merge commit message.
- The branch is pushed (`origin/wp-db-native-soft-fallback`) but the merge is
  local-only until maintainers approve a push. No PR exists yet.
- Orchestrator assumptions for the previously open decisions (API
  preservation, competing-implementation envelope, mandatory demo subset,
  native soft direction, opt-in adaptive activation) are recorded in
  `decisions.md`.

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
  feature, demo, and performance targets. It now includes concrete
  Kim/Pollard 2011 and Jain/Liu 2011 paper numbers, current DART 6 evidence,
  and acceptance gates for every representative row. The matrix is complete as
  WP-DB.03 evidence; the runnable scene, adaptive-contact, native-collision,
  and flagship-demo follow-through remains owned by WP-DB.05, WP-DB.08, and
  WP-DB.09.
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
- A follow-up WP-DB.06 inverse-dynamics slice is implemented locally:
  `SoftBodyNode::{updateAccelerationID,updateTransmittedForceID,updateJointForceID}()`
  now use `PointMassPhaseView` and shared cache checks instead of per-point
  helper calls, while preserving the existing DART 6 point external-force and
  joint-force semantics. Focused equation, allocation, and native checksum
  smokes passed after this slice.
- A follow-up WP-DB.06 phase-view span slice is implemented locally:
  `PointMassPhaseView` now stores raw point/state/property spans and a cached
  count instead of vector references, while `updateArtInertia()` uses the
  simplified point-mass scalar formula for explicit and implicit `Pi`. Focused
  soft dynamics, allocation, and native checksum smokes passed after this
  slice, but parent/base benchmark rows remain noisy and this is not a final
  speedup claim.
- A follow-up WP-DB.06 fused aggregation slice is implemented locally:
  several cached point-force fill loops now accumulate their parent soft-body
  spatial contribution in the same pass, and `updateBiasImpulse()` skips the
  point loop that only added freshly zeroed `mImpBeta` values. Focused soft
  dynamics, allocation, and native checksum smokes passed after this slice.
  The later benchmark-correction slice below records the current committed
  parent/base comparison artifact.
- A follow-up WP-DB.06 benchmark-correction slice is implemented locally in
  commit `649926d28dc`: the branch restores the prior two-pass
  `updateArtInertia()` scalar timing after same-host comparisons showed the
  simplified point-mass `Pi` formula and direct cached-position variants could
  regress FCL rows. The retained code keeps the span-backed phase view and a
  conservative `updateBiasForce()` connection-loop cleanup. Focused
  `test_SoftDynamics`, `INTEGRATION_StepAllocation`, FCL `soft_cubes`, and
  native `soft_bodies` checksum smokes passed. The comparison artifact
  `.benchmark_results/wp-db06-inertia-conn-649926-parent-43347c-base/` reports
  evaluator `PASS`, checksum-equivalent FCL/native correctness scenes, and
  native as the winner for every tracked current scene/thread row. A few
  current-vs-parent/base CPU mean rows remain noise-sensitive, so this is
  evidence for the correction and native-detector ranking, not final
  all-threshold completion.
- A follow-up WP-DB.06 soft aggregation temporary cleanup is implemented
  locally in commit `221fdf00145`: `SoftBodyNode` now avoids dynamic diagonal
  matrices in augmented-mass aggregation, writes gravity projection directly
  into the destination segment, skips the DART 6 no-op point inverse-mass
  aggregation dispatch, sums point masses through contiguous properties, and
  clears point external/internal forces through direct loops. Focused
  soft-dynamics/allocation tests, FCL/native checksum smokes, and `pixi run
  lint` passed. The comparison artifact
  `.benchmark_results/wp-db06-aggregation-temp-221fdf-parent-423f926-base/`
  reports evaluator `PASS`, checksum-equivalent FCL/native correctness scenes,
  and native as the winner for every tracked current scene/thread row. The
  broad run still had one positive current-vs-parent CPU mean on
  `dart/soft_open_chain/1`, but targeted reruns of that exact row measured
  current faster than parent in both parent-then-current and current-then-parent
  order. Keep final all-threshold claims gated on an exclusive idle-host rerun.
- A follow-up WP-DB.08 native primitive-frame and stability-gate slice is
  implemented locally: native soft-vs-plane, soft-vs-sphere, and soft-vs-box
  contacts classify point masses in primitive-local coordinates and compute
  world contact points only after a vertex is known to collide. The
  representative `test_SoftDynamics` finite-state and one-thread versus
  four-thread final-state gate now runs under both the default detector and
  `CollisionDetectorType::Dart`. Focused
  `test_DARTCollisionDetector`, `test_SoftDynamics`, and
  `INTEGRATION_StepAllocation` gates passed, `drop_box` FCL/native 200-step
  checksums still matched exactly, native `soft_cubes`, `soft_bodies`, and
  `soft_open_chain` 200-step checksums matched between `THREADS=1` and
  `THREADS=16`, and a quick current-only benchmark kept native ahead of FCL on
  every tracked scene/thread row. This closes the explicit native
  `test_SoftDynamics` acceptance gate; fuller triangle/contact-neighborhood
  coverage and threshold-quality parent/base comparison remain open.
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
- A direct `NativeCollisionDetector` fallback slice is committed locally through
  `0ed32afba03` on branch `wp-db-native-soft-fallback`:
  `SoftMeshShape` and `EllipsoidShape` objects now keep native broadphase AABBs
  while routing fallback pairs through cached DART-native collision objects.
  Direct `COLLISION_DETECTOR=native` no longer skips the representative soft
  scenes, reuses scratch fallback results without lookup-cache bookkeeping, uses
  the cached plane fallback for plane/soft pairs, lazily creates the persistent
  native manifold cache, skips manifold-cache refresh for soft fallback groups,
  fuses native object update/AABB collection before broadphase refresh, and
  benefits from contiguous brute-force native broadphase storage. During review
  of the uncommitted changes, an unsafe ordered-AABB broadphase shortcut was
  found and replaced with an id-checked `updateRange()` path plus the
  `BruteForceBroadPhase.UpdateRangeHandlesOutOfOrderIds` regression.
- The current handoff file is
  `docs/dev_tasks/dart6_deformable_body_performance/HANDOFF.md`. It records the
  exact branch, commit, interrupted benchmark attempt, validation commands, and
  next takeover steps for a fresh agent.
- Focused validation after lint and before commit `0ed32afba03` passed:
  `pixi run lint`, the focused `cmake --build` for `soft_body_headless`,
  `BM_INTEGRATION_soft_body`, `UNIT_collision_native_detector_adapter`,
  `UNIT_collision_native_brute_force`, `test_DARTCollisionDetector`,
  `test_SoftDynamics`, and `INTEGRATION_StepAllocation`, and the focused CTest
  regex covering those five tests.
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
- Branch `wp-db-native-soft-fallback` is local-only at this handoff: no upstream
  and no GitHub PR was attached. It was six commits ahead of
  `origin/release-6.20` before this docs handoff. Because it has not been
  published, a future agent may clean up/squash the branch history before first
  push, but only after preserving evidence and rerunning the required gates.
- A formal current/parent/base benchmark was started with output directory
  `.benchmark_results/wp-db08-native-soft-fallback-0ed32af-parent-40445e-base-2ba736`
  and then killed after the user requested a handoff-only stop. It created
  current/parent/base metadata and cold build worktrees but no usable
  `summary.md`, `raw/`, or `logs/` timing evidence. Do not cite that aborted
  attempt as benchmark proof.

Next steps:

1. Start by reading `HANDOFF.md`, verifying `git status -sb`, checking whether
   `origin/release-6.20` has moved, and confirming no previous benchmark
   process remains alive.
2. Re-run the current/parent/base comparison on an idle host with
   `--detectors fcl,dart,native,bullet,ode`, `--threads 1,16`, and the default
   expected fastest detector (`native`). Prefer a fresh output directory such as
   `.benchmark_results/wp-db08-native-soft-fallback-0ed32af-parent-40445e-base-2ba736-rerun`.
   Refresh `06-pr-evidence.md` only after the generated `summary.md` proves
   checksum eligibility, direct-native winner rows, and current-vs-parent/base
   CPU-change details. Use the generated graph sections in the PR description.
3. If the comparison fails, inspect the `summary.md` and raw logs first, then
   optimize the native path generally rather than overfitting one row. Re-run
   `pixi run lint`, focused native/soft tests, and the comparison on the final
   commit.
4. Use `soft_body_headless` for longer single-core and multi-core profile
   captures on an idle host, then choose the next measured soft-body layout
   slice. The likely next WP-DB.06 slice is retained SoA scratch for
   point-mass phase inputs before any `dart/simd/` kernel.
5. Extend `test_SoftDynamics` beyond default/native finite-state and
   one-thread versus four-thread final-state checks with energy, contact-force,
   CoP, or historical-golden regression thresholds that are stable across
   supported platforms.
6. Continue WP-DB.08 with fuller triangle/contact-neighborhood coverage and
   stronger multicore scaling beyond the current pair-level soft-soft worker
   path before preferring native as the default soft collision backend.
7. Before creating the PR, capture a same-host baseline-vs-branch performance
   comparison and include newly added GUI example commands plus locally captured
   videos as PR evidence. The first evidence packet exists in
   `06-pr-evidence.md`; rerun it on a cleaner host before turning the smoke
   rows into PR threshold claims.

Do not mark this task complete until every work packet in `README.md` has
current evidence or a maintainer-approved deferral in a durable owner doc.
