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
- `FCLCollisionObject::updateEngineData()` replaces the current FCL soft-mesh
  geometry with the same topology instead of using FCL's dynamic update path,
  which refits swept previous/current vertex bounds that DART's discrete
  soft-body collision does not consume.
- The shared-vertex path replaces one FCL vertex per point mass instead of
  rewriting every triangle; the triangle-contiguous path keeps the small
  articulated-soft-link specialization and replaces those triangles in place.
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

A later same-host `soft_bodies` FCL smoke over 200 steps kept the exact
step-200 checksum while replacing FCL's swept dynamic refit path with the
topology-preserving replace path. The measured elapsed time changed from
88.524 ms to 58.334 ms on that run; treat the exact number as load-sensitive,
but the profile showed the removed refit path was the dominant cost.

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

## Contiguous impulse/constrained-term slice

The next WP-DB.06 implementation removes another small layer of per-point
dispatch from the impulse and constraint-resolution phases:

- `SoftBodyNode::updateBiasImpulse()` now fills each point mass's impulse
  caches directly instead of calling the protected per-point helper.
- `SoftBodyNode::updateTransmittedImpulse()` reads point masses through an
  indexed loop and uses the owning soft body's contiguous point-property vector
  for mass values.
- `SoftBodyNode::updateConstrainedTerms()` updates the contiguous point-state
  vector directly for velocity, acceleration, and generalized-force changes,
  then preserves the same velocity and acceleration dirty-notice behavior that
  the old `PointMass` setters provided.

This is another scalar data-access cleanup, not a new algorithm. It reduces
point-mass method dispatch and repeated state/property lookups in phases that
already execute under the parent soft body's recursive dynamics update. The
remaining storage problem is unchanged: point masses are still heap-owned
objects behind `std::vector<PointMass*>`, and SIMD still needs a retained
phase-input facade or SoA storage before it should be introduced.

A native `soft_bodies` 200-step smoke after this slice preserved identical
step-200 checksum rows for `THREADS=1` and `THREADS=4`:
`skelPosL1=1.3242580728329107`,
`pointPosL1=0.12844450735027885`,
`pointVelL1=6.7481899526541707`, and
`pointWorldPosL1=189.54545901295526`. The elapsed rows were 25.346 ms and
23.295 ms respectively on this host; treat them as smoke timing, not a stable
threshold.

## Internal phase-view slice

The next WP-DB.06 implementation adds a `.cpp`-local
`PointMassPhaseView<StateVector>` facade for recursive soft-body dynamics
loops. It does not change public headers, class layout, or the existing
`PointMass*` ownership model. The view centralizes the invariant that
`mPointMasses`, `mPointStates`, and `mPointProps` have matching lengths, then
gives each phase a single access shape for point objects, contiguous state, and
contiguous properties.

The converted loops are:

- `updateTransform`,
- `updateVelocity`,
- `updatePartialAcceleration`,
- `updateArtInertia`,
- `updateBiasForce`,
- `updateAccelerationFD`,
- `updateTransmittedForceFD`,
- `updateVelocityChangeFD`,
- `updateTransmittedImpulse`, and
- `updateConstrainedTerms`.

This is the narrow facade step before a retained SoA scratch or object-storage
rewrite. It reduces duplicated size checks and makes the phase inputs explicit,
but it still dereferences the existing point-mass object vector for cached
per-point dynamics fields.

Focused verification after this slice:

- `test_SoftDynamics` passed.
- `StepAllocation.*Soft*` passed all 12 rows with zero `operator new`, zero raw
  `malloc`, and zero counted base allocator growth in the measured windows.
- Native `soft_bodies` 200-step checksum rows matched exactly between
  `THREADS=1` and `THREADS=4`:
  `skelPosL1=1.3242580728329107`,
  `pointPosL1=0.12844450735027885`,
  `pointVelL1=6.7481899526541707`, and
  `pointWorldPosL1=189.54545901295526`.

## Matrix/vector aggregation phase-view slice

The next WP-DB.06 implementation extends the phase-view pattern to matrix and
vector aggregation paths that previously delegated into per-point helper
methods:

- `updateMassMatrix`,
- `aggregateMassMatrix`,
- `aggregateAugMassMatrix`,
- `updateInvMassMatrix`,
- `aggregateGravityForceVector`,
- `updateCombinedVector`,
- `aggregateCombinedVector`, and
- `aggregateExternalForces`.

The moved formulas are behavior-preserving cache fills. `SoftBodyNode` now
uses the same phase view to read contiguous point state/property vectors while
writing each point mass's existing cached `mM_*`, `mG_F`, `mCg_*`, and inverse
mass helper fields. The external-force path also skips the old no-op
`PointMass::aggregateExternalForces()` loop and projects point external forces
directly from the parent soft body after a shared transform-cache check.

Focused verification after this slice:

- `test_SoftDynamics` passed, including representative public
  matrix/vector-equation checks.
- `StepAllocation.*Soft*` passed all 12 zero-allocation rows.
- Native `soft_bodies` 200-step checksum rows matched exactly between
  `THREADS=1` and `THREADS=4` with the same step-200 values as the prior
  phase-view slice.

## Inverse-dynamics phase-view slice

The next WP-DB.06 implementation extends the same phase-view pattern to the
remaining point-mass inverse-dynamics helpers used by `SoftBodyNode`:

- `updateAccelerationID`,
- `updateTransmittedForceID`, and
- `updateJointForceID`.

The moved formulas preserve the existing DART 6 semantics:
`updateTransmittedForceID()` still subtracts each point mass's local external
force independent of the parent method's `_withExternalForces` body-wrench
switch, and `updateJointForceID()` still copies each cached point force back
into the point state without adding the TODO spring/damping terms. The parent
soft body now refreshes shared transform, velocity, partial-acceleration, and
acceleration caches once before the point loop rather than relying on per-point
getter checks.

Focused verification after this slice:

- `test_SoftDynamics` passed.
- `StepAllocation.*Soft*` passed all 12 zero-allocation rows.
- Native `soft_bodies` 200-step checksum rows matched exactly between
  `THREADS=1` and `THREADS=4` with the same step-200 values as the prior
  phase-view slice.

## Span-backed phase-view slice

The next WP-DB.06 implementation keeps `PointMassPhaseView` `.cpp`-local but
changes it from vector references to a small raw span:

- cached `PointMass* const*`, point-state pointer, point-property pointer, and
  count,
- invariant checks remain at construction, and
- converted hot loops index contiguous state/property storage without asking
  the owning vectors for `size()` or `data()` on every pass.

The same slice simplifies the point-mass articulated-inertia scalar path.
Explicit `Pi` is mathematically zero for a point mass, and implicit `Pi` now
reuses the already computed inverse denominator as
`mass * implicitOffset * implicitPsi` instead of the equivalent old expression
`mass - mass^2 * implicitPsi`. This keeps the existing scalar formula but
removes one multiply and one subtract per point-mass inertia update.

Focused verification after this slice:

- `test_SoftDynamics` passed.
- `INTEGRATION_StepAllocation` passed.
- `StepAllocation.*Soft*` passed all 12 zero-allocation rows.
- Native `soft_bodies` 200-step checksum rows matched exactly between
  `THREADS=1` and `THREADS=4` with the same step-200 values as the prior
  phase-view slice.
- Current-only benchmark repeats showed lower native CPU time than the earlier
  pre-span current summary on most tracked rows, but parent/base threshold
  evidence remains open and noisy enough that this slice is not a final
  speedup claim.

## Fused point-force aggregation slice

The next WP-DB.06 implementation removes several remaining two-pass
point-mass aggregation patterns inside `SoftBodyNode.cpp`. Paths that used one
loop to fill a point-mass cached force and a second loop to add the point
contribution back into the parent soft-body spatial vector now compute and
accumulate in the same point loop:

- inverse dynamics transmitted force,
- forward dynamics bias force,
- mass and augmented-mass aggregation,
- inverse-mass bias aggregation,
- gravity-force aggregation, and
- combined-vector aggregation.

The slice also drops the point-mass pass in `updateBiasImpulse()` because
`mImpBeta` is reset to zero immediately before the removed aggregation, so the
old loop only added zeros. Where the fused loops read cached point local
positions directly, the code now performs one explicit transform preflight
instead of relying on `PointMass::getLocalPosition()` to check the same dirty
flag for every point.

Focused verification after this slice:

- `test_SoftDynamics` passed.
- `INTEGRATION_StepAllocation` passed.
- `StepAllocation.*Soft*` passed all 12 zero-allocation rows.
- Native `soft_bodies` 200-step checksum rows matched between `THREADS=1` and
  `THREADS=4`.
- Current-only benchmark repeats improved most rows relative to the prior
  span-backed current binary. The later articulated-inertia correction below
  records the current committed parent/base comparison artifact.

## Articulated-inertia timing correction

Commit `649926d28dc` keeps the span-backed phase view, but it reverts the
simplified point-mass `Pi` formula from the span-backed slice. Same-host
current/parent/base comparisons showed that the algebraically smaller formula
and direct cached-position variants could move FCL rows in the wrong direction,
especially `soft_cubes`, even though focused checksums stayed unchanged.

The current `updateArtInertia()` therefore restores the prior DART 6 timing:

- compute the explicit and implicit articulated-inertia scalars first with the
  old expressions,
- preserve the lazy `PointMass::getLocalPosition()` transform refresh in the
  second pass that contributes point inertia to the parent soft body, and
- keep the change local to `SoftBodyNode.cpp` without public header or class
  layout changes.

The same commit keeps a conservative `updateBiasForce()` cleanup that hoists
the connected-neighbor index pointer and velocity scale out of the inner spring
loop. This avoids repeated vector access and scalar recomputation without
changing the point-mass force model or cache invalidation behavior.

Focused verification after this correction:

- `pixi run lint` passed before the commit.
- `test_SoftDynamics` and `INTEGRATION_StepAllocation` passed.
- `soft_body_headless soft_cubes 200 200` with `COLLISION_DETECTOR=fcl`
  and `THREADS=1` preserved the expected checksum.
- `soft_body_headless soft_bodies 200 200` with `COLLISION_DETECTOR=dart`
  and `THREADS=16` preserved the expected checksum.
- `.benchmark_results/wp-db06-inertia-conn-649926-parent-43347c-base/`
  records a two-cycle current/parent/base comparison. The evaluator verdict is
  `PASS`, FCL and native are checksum-equivalent on the correctness scenes,
  and the native detector wins every tracked current scene/thread row. A few
  current-vs-parent/base CPU mean rows remain within host-load noise, so this
  is not final threshold-quality evidence; rerun on an exclusive idle host
  before claiming every row beats the base and parent.

## Soft aggregation temporary cleanup

Commit `221fdf00145` removes a small set of remaining soft-body aggregation
overheads that were still visible after the phase-view work:

- `SoftBodyNode::getMass()` now sums contiguous point properties directly
  instead of pointer-chasing each `PointMass`.
- `aggregateAugMassMatrix()` no longer allocates dynamic diagonal stiffness and
  damping matrices or a temporary acceleration vector for the soft-body row; it
  writes the Jacobian product into the destination block and adds scalar
  spring/damping terms in place.
- `aggregateGravityForceVector()` mirrors the heap-free `BodyNode` pattern by
  assigning the Jacobian product directly into the destination segment and
  negating in place.
- `aggregateInvMassMatrix()` skips the per-point dispatch to
  `PointMass::aggregateInvMassMatrix()`, which is an intentional no-op in
  DART 6.
- `clearExternalForces()` clears point external forces in one direct loop and
  dirties the skeleton once when any point force was nonzero.
- `clearInternalForces()` zeros the contiguous point-state force entries
  directly.

Focused verification after this slice:

- `pixi run cmake --build build/default/cpp/Release --target
  test_SoftDynamics INTEGRATION_StepAllocation soft_body_headless
  BM_INTEGRATION_soft_body --parallel 8` passed.
- `pixi run ctest --test-dir build/default/cpp/Release -R
  'test_SoftDynamics$|INTEGRATION_StepAllocation$' --output-on-failure`
  passed.
- FCL `soft_cubes` and native threaded `soft_bodies` 200-step checksum smokes
  preserved the expected step-200 values.
- `pixi run lint` passed before the commit.
- `.benchmark_results/wp-db06-aggregation-temp-221fdf-parent-423f926-base/`
  records a current/parent/base comparison with evaluator `PASS`, FCL/native
  checksum equivalence on the correctness scenes, and native `dart` as the
  winner for every tracked current scene/thread row.
- The broad comparison still reported a positive mean on
  `dart/soft_open_chain/1` against the parent. Targeted reruns of that exact
  row using the already-built parent/current binaries contradicted the broad
  positive: parent-then-current measured 4.851 ms vs 4.795 ms CPU mean, and
  reverse order under higher load measured 7.662 ms vs 7.420 ms. Treat the
  broad positive as noise, but keep final threshold claims gated on an
  exclusive idle-host rerun.

## Rest-detection memory-hardening carryover

After refreshing `origin/dart6-memory-hardening`, this branch first ported its
narrow `Skeleton::checkExternalDisturbanceAndReset()` body-wrench scan:

- The rest-detection query now scans `BodyNode::getExternalForceLocal()`
  directly instead of materializing `Skeleton::getExternalForces()` when the
  external-force projection cache is dirty.
- `_resetCommand` still observes body wrenches that project to zero generalized
  force.
- Generalized force and command scans remain per-DOF and allocation-free.

This is a shared simulation allocation-hardening slice, not the final soft-body
storage answer. It removes an avoidable projection-cache materialization from a
per-step quiet/rest query while keeping the existing DART 6 public API. After
the full `dart6-memory-hardening` stack was merged locally, the conflict
resolution also preserved that branch's point-mass external-force scan, so
rest detection stays allocation-aware for both rigid body wrenches and soft
point-mass forces.

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

- Stack on `origin/dart6-memory-hardening` instead of adding an unrelated
  soft-body allocator path.
- Use its `World`/`MemoryManager`/`FrameAllocator` surfaces for soft-body
  allocation gates and later point-mass storage work.
- If `dart6-memory-hardening` lands before this branch, refresh against the
  release branch and rerun the soft allocation and checksum gates before
  claiming the stack resolved cleanly.
- Keep the timing evidence conservative. The allocator stack is a correctness
  and per-step allocation hardening dependency; it is not by itself a stable
  speedup claim.

## Native soft allocation gate

The local branch now adds native soft-body coverage to
`tests/integration/test_StepAllocation.cpp`. The first test scene builds a
3x3x3 soft box over a ground box, forces `CollisionDetectorType::Dart`, and
uses the memory-hardening branch's counted world allocator and frame scratch
surfaces. A follow-up scene stacks two 3x3x3 soft boxes on the ground so the
measured window exercises native soft-soft contacts as well as soft-ground
contacts. A third gate loads `dart://sample/skel/softBodies.skel`, transfers
the parsed soft skeletons into a counted native world without cloning
`SoftMeshShape`, and measures a representative SKEL-authored soft-dynamics
window. That SKEL window has no contacts in the measured range, so it is an
allocation gate for soft-body stepping rather than a contact-solving gate.
Follow-up gates load `dart://sample/skel/soft_open_chain.skel` for another
SKEL-authored no-contact soft-dynamics window and
`dart://sample/skel/soft_cubes.skel` for a contact-producing SKEL-authored
window.

The gate measures both explicit and implicit integration after a warmup:

```bash
./build/default/cpp/Release/tests/integration/INTEGRATION_StepAllocation --gtest_filter='StepAllocation.*Soft*'
```

Current result on 2026-07-07:

| Test row | Steps | Contacts | Soft-soft contacts | `operator new` | Raw `malloc` | Base allocator growth |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Soft-box explicit first post-bake step | 1 | 9 | 0 | 0 | 0 | 0 |
| Soft-box implicit second step | 1 | 9 | 0 | 0 | 0 | 0 |
| Soft-box explicit first post-bake raw gate | 1 | 9 | 0 | 0 | 0 | 0 |
| Soft-box implicit second-step raw gate | 1 | 9 | 0 | 0 | 0 | 0 |
| Soft-stack steady-state gate | 100 | 27 | 18 | 0 | 0 | 0 |
| Soft-stack steady-state raw gate | 100 | 27 | 18 | 0 | 0 | 0 |
| `softBodies.skel` steady-state gate | 100 | 0 | 0 | 0 | 0 | 0 |
| `softBodies.skel` steady-state raw gate | 100 | 0 | 0 | 0 | 0 | 0 |
| `soft_open_chain.skel` steady-state gate | 100 | 0 | 0 | 0 | 0 | 0 |
| `soft_open_chain.skel` steady-state raw gate | 100 | 0 | 0 | 0 | 0 | 0 |
| `soft_cubes.skel` steady-state gate | 100 | 9 | 0 | 0 | 0 | 0 |
| `soft_cubes.skel` steady-state raw gate | 100 | 9 | 0 | 0 | 0 | 0 |

The implementation changes behind the gate are intentionally local to
steady-state allocation behavior:

- `SoftContactConstraint` keeps fixed one-contact storage, fixed three-row
  Jacobian storage, and a fixed 3x2 tangent basis instead of allocating
  vectors or dynamic Eigen matrices per contact.
- `ConstraintSolver` reuses soft contact constraints across steps and resets
  them with the current contact and timestep.
- `World::updateRestStates()` uses retained scratch vectors for the deep and
  supported initial-contact skeleton sets, and the reserve path sizes those
  vectors during simulation preparation.
- `World::updateAllRestingKinematicSnapshot()` reuses retained snapshot
  storage and copies generalized positions by scalar index, avoiding
  per-step temporary `Eigen::VectorXd` allocations for no-contact soft scenes.

This gate proves the current native soft-box contact lane can step without
heap growth after preparation, and that a small native soft-soft stack can run
steady-state steps without heap growth after warmup. It also proves
representative `softBodies.skel` and `soft_open_chain.skel` no-contact
soft-dynamics windows and a contact-producing `soft_cubes.skel` window are
heap-free after warmup. It does not yet prove contiguous point-mass object
storage or SIMD-friendly SoA layout.

## Retained SoA mirror: measured reject (2026-07-10)

A GPT-5.5 (xhigh) executor implemented and measured the "retained SoA scratch
for phase inputs" candidate (sequence step 2 below) on the reunified branch:
a `.cpp`-private `PointMassNotifier` subclass carried retained SoA mirrors of
point positions, velocities, masses, local positions, and flattened
connectivity for `updateBiasForce`/`updateArtInertia`, in three variants
(full mirror, guarded topology refresh, narrow mirror).

Result: **rejected on measurement**. Baseline `soft_bodies` 200-step
single-thread native-lane elapsed median was 13.691 ms (5 runs, loaded
host); the variants measured 17.551 ms, 18.558 ms, and 17.284 ms — all
consistently worse, because the per-step mirror copy costs more than the
pointer-chasing it removes at these point counts. One variant also initially
violated the `INTEGRATION_StepAllocation` soft gates through unconditional
profiler scopes. All candidate code was reverted; lint, the focused CTest
battery (5/5), the full suite (151/151), and the 30-row checksum battery
(bit-identical) passed after the revert.

Consequence for the sequence below: skip separate per-phase mirrors. The
next measured step is real contiguous point-mass storage (step 4) or
producer-phase-maintained scratch, evaluated on an idle host with profiler
scopes present in both baseline and candidate builds.

## Soft-phase profiler scopes and contiguous-storage disposition (2026-07-11)

The merged `SoftBodyNode.cpp` carried no profiler scopes, so phase-level
claims could not be measured. Conditional
`DART_PROFILE_SCOPED_IF_N(isProfileRecordingEnabled(), ...)` scopes now cover
`updateBiasForce`, `updateArtInertia`, `updateTransform`, and
`updateVelocity` — no-ops when the profiler is not recording, so the
`INTEGRATION_StepAllocation` windows stay allocation-free (the unconditional
form measured by the earlier executor tripped them). First captured rows on
the native-lane `soft_bodies` 200-step run: `updateBiasForce` 4.88 ms
(59.3% recorded share, 1000 calls) with nested `updateArtInertia` 3.12 ms
(37.9%), `updateVelocity` 0.45 ms, `updateTransform` 0.27 ms — on this lane
the point-mass dynamics phases, not collision refresh, now dominate.

Contiguous point-mass object storage (sequence step 4) is parked with an
evidence-backed disposition rather than implemented: `PointMass` has a
virtual destructor (vtable pointer in every object), and
`configurePointMasses` burst-allocates all point masses of a body in one
loop, which general-purpose allocators already lay out near-contiguously.
Combined with the retained-mirror measured reject above (copy cost dominates
at these point counts), an arena is unlikely to pay for its lifecycle
complexity (clone, shrink, addPointMass growth with address stability).
Revisit only if the new profiler rows show pointer-chase stalls dominating a
phase after the activation and native-kernel packets land.

1. Extend the internal phase view into retained SoA scratch that can expose
   contiguous spans of point positions, velocities, accelerations, forces,
   masses, and connectivity without changing public `PointMass*` accessors.
2. Keep existing `PointMass` objects initially, but mirror the hottest phase
   inputs into retained scratch so `updateBiasForce` and `updateArtInertia` can
   be measured as contiguous scalar loops.
3. Once scalar checksums match, evaluate replacing per-point allocations with a
   contiguous object pool or memory-manager-backed block allocation.
4. Add SIMD kernels only for phases whose memory layout is contiguous and whose
   FP contract is approved.
