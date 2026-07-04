# WS-C — Dynamics batching + allocation lane

gz scenes are thousands of single-body FreeJoint skeletons. Round 1 added
a FreeJoint *integration* fast path (#3193) and thread-parallelized many
passes, but per-skeleton work is not data-batched: repeated per-step
`dynamic_cast`s (`World.cpp:309`, `Skeleton.cpp:4282`,
`BoxedLcpConstraintSolver.cpp`), no single-free-body fast path in
`Skeleton::computeForwardDynamics`, AoS structures rebuilt per step, and
per-step heap traffic in the #3227 shallow-support machinery
(`World.cpp:159-174`, `:298`). DART 7's assessment names
zero-steady-state-allocation as the single scaling enabler (the MuJoCo
lesson).

Version-semantics constraint for the whole lane: kinematic/velocity
version counters drive the deactivation snapshots and all-resting fast
path (`World.cpp:318-319`); any batch path that writes state **must** bump
the same versions/dirty flags the scalar path does, or sleeping breaks.

#### WP-PG.30 — Single-free-body classification cache + FD fast path

- Status: open
- Objective: (a) cache an `is single free body` classification per
  skeleton keyed on structural version (killing the per-step
  dynamic_casts at the three sites above); (b) add the missing
  single-free-body fast path to `Skeleton::computeForwardDynamics`
  (mirroring `Skeleton.cpp:4279-4295`'s existing pattern).
- Value: O(skeletons) RTTI and virtual-pass overhead removed on exactly
  the gz workload shape; enabler for WP-PG.12 and WP-PG.33.
- Prior art (mandatory reading): round 1 measured and REJECTED a narrow
  variant — skipping the FreeJoint `updateJointForceFD()` no-op for
  one-body roots regressed RTF to 0.2497 (journal on
  `origin/perf/dart6-single-reactive-raw-root`); the related joint-cache
  and group-cache experiments also failed (see the prior-art inventory in
  01-baseline-evidence.md). This packet's classification-cache approach
  must be measured against those outcomes, not assumed better.
- Scope: `dart/dynamics/Skeleton.*`, `dart/simulation/World.cpp`,
  consumer sites in the solver; additive private state only (pre-6.20.0
  packaging window) or cpp-side cache.
- Non-goals: articulated-skeleton FD changes; public API.
- Acceptance evidence: bit-identical hashes on all guard scenes and
  detectors; step-time delta on 900/3000-object scenes.
- Dependencies: WP-PG.01.

#### WP-PG.31 — Scratch retention for shallow-support machinery

- Status: open
- Objective: reuse member buffers for
  `findShallowSupportedFreeRoots`' map+vector and
  `snapshotFreeRootVelocities`' vector, and skip the pass entirely when
  no skeleton qualifies (using WP-PG.30's classification).
- Value: removes per-step allocations added by #3227 on every world;
  zero behavior risk.
- Scope: `dart/simulation/World.*` cpp + private members.
- Acceptance evidence: bit-identical everything; allocation-count delta
  (measure with the WP-PG.32 gate tooling if landed, else heaptrack row).
- Dependencies: WP-PG.30 (for the skip), else standalone buffer reuse.

#### WP-PG.32 — Frame-arena allocation discipline + CI allocation gate

- Status: open
- Objective: port the *pattern* of main's `FrameAllocator`
  (`dart/common/frame_allocator.hpp`) as an additive `dart::common`
  utility; route ConstraintSolver/collision per-step scratch through it;
  expose capacity/overflow counters; add an allocation-count regression
  gate to CI (modeled on main #3103/#2504) using the guard scenes.
- Value: converts "we think steady-state allocates" into a ratcheted
  invariant; unlocks safe scratch reuse everywhere else.
- Scope: new `dart/common` header+cpp (additive, not in any public class
  layout), solver/collision cpp plumbing, CI job/test.
- Non-goals: replacing std allocators globally; touching Eigen allocator
  behavior (eigen-overalignment gate applies).
- Acceptance evidence: bit-identical outcomes; steady-state
  allocations-per-step counter on guard scenes recorded and gated;
  `pixi run test-eigen-overalignment` green.
- Dependencies: WP-PG.01; pairs well after WP-PG.31.

#### WP-PG.33 — SoA batched integration for single-free-body cohorts

- Status: gated (needs WP-PG.30 + D1/D2 outcomes if SIMD-backed)
- Objective: batch position/velocity integration for the classified
  single-free-body cohort through SoA buffers (pimpl/solver-internal
  scratch), optionally using `dart/simd` kernels from WS-D; preserve
  per-skeleton version/dirty-flag semantics exactly.
- Value: converts thousands of virtual per-skeleton updates into a few
  vectorized loops — the round-2 shot at the integration share.
- Scope: `dart/simulation/World.cpp` step path + pimpl scratch;
  scalar-first implementation with SIMD as a swap-in.
- Non-goals: SoA state storage in public classes (explicit non-goal in
  README); articulated skeletons.
- Acceptance evidence: bit-identical hashes scalar-vs-batched (scalar
  math first; SIMD under the D1 contract); guard-scene matrix; sleeping
  regression coverage (bodies must still fall asleep identically —
  deactivation counters in evidence table).
- Dependencies: WP-PG.30; WS-D WP-PG.40/41 for the SIMD variant.
