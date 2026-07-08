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

- Status: done — #3310 (`wp-pg-30-single-free-body-cache`)
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
- Completion evidence: merged as PR #3310. A/B reported S5 -12.2%, S4 -5.3%,
  S3 -2.3%, solve-bound rows flat, with 8/8 guard hashes bit-identical. The
  FD fast path was skipped with an in-code rationale because round-1 evidence
  showed the narrower FreeJoint no-op skip regressed.
- Dependencies: WP-PG.01.

#### WP-PG.31 — Scratch retention for shallow-support machinery

- Status: local verified — `wp-pg-31-shallow-support-scratch`
- Objective: current `release-6.20` already retains the
  `findShallowSupportedFreeRoots` and `snapshotFreeRootVelocities` scratch
  buffers; finish the packet by skipping the shallow-support snapshot/find pass
  entirely when no mobile root-FreeJoint skeleton can use it.
- Value: removes unconditional per-step work added by #3227 on articulated or
  fixed-root worlds while preserving the existing cleanup pass that clears stale
  shallow-support state after topology/mobile-state changes.
- Scope: `dart/simulation/World.*` helper and step-path call sites.
- Acceptance evidence: current-base A/B artifact
  `/tmp/wp_pg31_ab_20260707T184319` on base `3964108a675` vs branch
  `21f691311df`: `double_pendulum.world` hashes identical
  (`0x1db838038acbd960`) with median step time 0.002106 -> 0.001836 ms
  (DART) and 0.001903 -> 0.001644 ms (ODE); generated 120-object DART/ODE
  guard hashes identical (`0x3bd9d26da5ea002b`,
  `0x5576f6244b736aae`); `BM_ContactContainerActive/120/1/{1,16}` ODE medians
  6588 -> 6433 ms and 6998 -> 6468 ms; base and branch both passed 30/30
  default 16-thread crash stressors.
- Dependencies: WP-PG.30.

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
