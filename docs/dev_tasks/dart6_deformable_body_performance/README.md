# DART 6 deformable body feature and performance

Owner surface for restoring DART 6 deformable-body simulation quality without
using DART 7 clean-break APIs as the implementation target.

## Objective

Make DART 6 deformable-body simulation feature-complete, stable, and fast
enough to beat representative competing implementations and at least match the
published real-time or near-real-time targets from the reference papers on CPU.

## Reference scope

- Kim and Pollard, "Fast Simulation of Skeleton-driven Deformable Body
  Characters" (ACM TOG 30(5), 2011): reduced nonlinear FEM, two-way
  skeleton/deformable/environment coupling, linear-time skeleton dynamics,
  explicit integration, and real-time or near-real-time character demos.
- Jain and Liu, "Controlling Physics-Based Characters Using Soft Contacts"
  (SIGGRAPH Asia 2011): point-mass surface flesh attached to articulated rigid
  bodies, vertex and edge springs, adaptive active surface vertices near
  contact, and LCP contact/friction coupling for controller robustness.

Current DART 6 `SoftBodyNode` resembles the Jain/Liu point-mass surface model,
but it does not yet prove the full coupled equations, adaptive activation,
contact correctness, or performance envelope.

## Compatibility envelope

- Target branch: `release-6.20`.
- Preserve existing public headers and DART 6 ABI/API behavior unless a
  maintainer explicitly accepts a breaking change.
- GPU is not a DART 6 implementation target. Tracked release-branch evidence
  only identifies GPU offload as DART 7-only; this task is CPU-first.
- Performance targets must cover:
  - single-core CPU baseline (`World::setNumSimulationThreads(1)`),
  - multi-core CPU scaling (`World::setNumSimulationThreads(16)` or host-capped
    equivalent),
  - SIMD opportunities through `dart/simd/` and compile-time feature selection.

## Current evidence

- `dart/dynamics/SoftBodyNode.*` stores point masses, faces, vertex/edge
  stiffness, and damping.
- `dart/dynamics/PointMass.*` implements transform, velocity, acceleration,
  implicit point-mass bias, and contact impulse paths, but several matrix/vector
  aggregation routines remain TODO or commented out.
- `dart/dynamics/SoftBodyNode` currently stores `std::vector<PointMass*>`, so
  point-mass state is pointer-chased rather than contiguous. WP-DB.06 must
  treat data layout and allocator ownership as first-class performance work.
- `dart/constraint/SoftContactConstraint.*` supports soft-body contact plumbing
  through `SoftMeshShape` point selection and friction directions.
- `tests/integration/test_SoftDynamics.cpp` contains active finite-state,
  deterministic threaded final-state, native-detector, and representative
  equations-of-motion gates for deformable scenes. WP-DB.04 now covers
  point-mass mass-matrix, augmented-mass, inverse-mass,
  inverse-augmented-mass, gravity, combined-vector, external-force, and
  representative rigid-only/soft-only/mixed-world matrix/vector checks,
  recorded in `07-equation-correctness.md`.
- `data/skel/test/test_double_pendulum.skel` used one legacy `<soft>` tag that
  `SkelParser` ignored; WP-DB.01 converts it to `<soft_shape>`, and WP-DB.02
  adds explicit soft-box fragments so both soft links parse cleanly.
- GUI examples exist in `examples/soft_bodies` and `examples/mixed_chain`, but
  there was no headless soft-body benchmark before this task.
- WP-DB.01 now adds `BM_INTEGRATION_soft_body` and records smoke rows in
  `01-baseline-evidence.md`.
- WP-DB.03 now converts Kim/Pollard 2011 and Jain/Liu 2011 into a concrete
  parity ledger in `02-paper-parity-matrix.md`, including paper scene metrics,
  current DART 6 evidence, and acceptance gates. The matrix itself is complete;
  the runnable scene, adaptive-contact, native-collision, and flagship-demo
  follow-through remains owned by WP-DB.05, WP-DB.08, and WP-DB.09.
- WP-DB.06 groundwork adds `soft_body_headless`, a deterministic checksum and
  text-profiler runner for soft-body scenes. Initial profile evidence and the
  `origin/dart6-memory-hardening` dependency analysis are recorded in
  `04-data-layout-and-memory-hardening.md`.
- `06-pr-evidence.md` records the current same-host baseline-vs-branch
  benchmark smoke rows, native/FCL headless parity evidence, and the current
  GUI-video status for PR preparation.
- WP-DB.06 first optimization changes FCL soft meshes to retain shared vertex
  topology, prime FCL's previous-vertex update buffer at geometry creation, and
  skip BVH refits when local point-mass positions have not changed. It improves
  the representative `soft_bodies` 200-step headless profile while preserving
  deterministic checksums, but the remaining refit-heavy path is still the main
  CPU bottleneck.
- WP-DB.06 scalar/cache optimization keeps public `SoftBodyNode` and
  `PointMass` APIs intact while caching parent angular velocity, local gravity,
  stiffness, damping, contiguous point state/property vectors, and point mass
  scalars across the hot point-mass dynamics formulas. The latest slice removes
  connected-neighbor `PointMass*` lookups from the bias-force edge loop and
  validates shared point-mass kinematic and articulated-inertia caches once per
  soft body. Follow-up kinematics overloads pass contiguous point
  state/property entries and cached parent transform/velocity terms through
  `updateTransform`, `updateVelocity`, and `updatePartialAcceleration`.
  Follow-up impulse/constrained-term loops update cached point-mass impulse
  fields and contiguous point state/property vectors directly from
  `SoftBodyNode`. A `.cpp`-local `PointMassPhaseView` now centralizes the
  matching point-object/state/property invariant for the converted recursive
  dynamics, inverse-dynamics, and matrix/vector aggregation phases without
  changing public headers or class layout. Follow-up slices back that view with
  raw point/state/property spans and fuse several remaining cached point-force
  fill and soft-body spatial aggregation loops. A later benchmark correction
  rejected the simplified point-mass articulated-inertia scalar formula,
  restored the prior two-pass `updateArtInertia()` timing, and kept only a
  conservative connection-loop cleanup in `updateBiasForce()`. The latest
  same-host comparison is recorded in `06-pr-evidence.md`; it passes checksum
  equivalence and native-detector winner checks, but strict per-row CPU
  threshold claims still need an exclusive idle-host rerun. A follow-up
  aggregation-temporary cleanup removes avoidable heap-temporary and no-op
  point-loop work from soft-body mass/gravity/force aggregation paths while
  preserving checksums and allocation gates. The branch is now
  stacked on
  `origin/dart6-memory-hardening` and
  uses its `World`/`MemoryManager`/`FrameAllocator` surfaces for native soft
  allocation gates. The latest local gates prove the measured native soft-box
  post-bake steps, a two-soft-box native stack steady-state window, and the
  `softBodies.skel`, `soft_open_chain.skel`, and contact-producing
  `soft_cubes.skel` SKEL-authored windows perform zero `operator new`, zero
  raw `malloc`, and zero base allocator growth.
  Contiguous point-mass storage and SIMD remain open.
- WP-DB.08 native collision slices add dynamic `SoftMeshShape` bounds and native
  point-mass contacts for soft-vs-plane, soft-vs-box, soft-vs-sphere, and
  soft-vs-ellipsoid pairs, plus a first soft-vs-soft vertex-face lane. The
  soft-soft lane now uses a backend-internal `DARTCollisionObject` soft mesh
  cache for local vertices, point-to-face metadata, and precomputed triangle
  geometry, early face-side separation rejection before barycentric projection,
  retained soft-face BVH traversal once a penetrating candidate exists, and
  pair-level worker scheduling for eligible finite-finite soft pairs. The
  soft-vs-primitive lanes also reuse cached local vertices and representative
  face IDs. A follow-up primitive-frame slice classifies soft-vs-plane,
  soft-vs-sphere, and soft-vs-box point contacts in primitive-local
  coordinates and only computes world contact points for colliding vertices.
  A 200-step `drop_box` headless run matched FCL checksums exactly and ran
  faster with `COLLISION_DETECTOR=dart` on this host, and the broader
  `soft_bodies` native diagnostic now runs without shape-creation or
  unsupported-pair warnings.
  Broader scenes still need fuller triangle/contact-neighborhood coverage and
  stronger multicore scaling beyond small pair counts before native can replace
  FCL. Native soft allocation gates now cover the soft-box contact lane, a
  soft-soft stack steady-state window, and the `softBodies.skel` no-contact
  soft-dynamics window, plus `soft_open_chain` and contact-heavy `soft_cubes`
  SKEL-authored windows.
- A follow-up direct `NativeCollisionDetector` slice routes `SoftMeshShape` and
  `EllipsoidShape` pairs through cached DART-native fallback objects instead of
  skipping them, reuses a scratch fallback result without colliding-object
  lookup-cache bookkeeping, specializes fallback plane pairs through the cached
  plane path, and stores brute-force native broadphase entries contiguously.
  Commit `0ed32afba03` tightens that path by lazily creating the persistent
  native manifold cache, skipping manifold-cache refresh for soft fallback
  groups, fusing native object update/AABB collection, and preserving
  broadphase id/AABB correctness through an id-checked range update. Focused
  native broadphase, detector, soft-dynamics, DART-detector, and allocation
  gates passed after `pixi run lint`. Current direct
  `COLLISION_DETECTOR=native` soft-scene smokes are warning-free and
  checksum-equivalent to `COLLISION_DETECTOR=dart`; the final
  current/parent/base matrix still gates threshold claims. The 2026-07-09
  comparison attempt was intentionally stopped before timing rows were
  produced, so it is not benchmark evidence.

## Work packets

| Packet | Scope | Done when |
| --- | --- | --- |
| WP-DB.01 baseline harness | Add headless Google Benchmark rows for existing soft-body scenes across single-core and multi-core CPU. | Benchmark target builds; rows report sim seconds per second, point masses, soft bodies, and threads. |
| WP-DB.02 stability gate | Re-enable or replace `test_SoftDynamics` with finite-state and equations-of-motion checks for representative soft scenes. | Focused integration test fails on NaN/Inf, gross state blow-up, one-thread versus four-thread final-state divergence, or representative matrix/vector equation inconsistency. Current default/native-detector finite-state and deterministic final-state coverage is recorded in `03-stability-gate.md`; equation correctness coverage is recorded in `07-equation-correctness.md`. |
| WP-DB.03 paper parity matrix | Convert both papers into a scene/feature/performance checklist. | Every representative paper demo has an owner row, current DART 6 status, target evidence, and acceptance gate in `02-paper-parity-matrix.md`. Follow-through scene implementation remains in WP-DB.05, WP-DB.08, and WP-DB.09. |
| WP-DB.04 coupled equation correctness | Finish or replace stubbed point-mass mass, inverse-mass, gravity, combined-force, and external-force aggregation paths. | Point-mass mass-matrix, augmented-mass, inverse-mass, inverse-augmented-mass, gravity, combined-vector, external-force, and representative rigid-only/soft-only/mixed-world matrix/vector checks now have active regressions in `07-equation-correctness.md`. |
| WP-DB.05 adaptive contact activation | Implement Jain/Liu-style active vertex neighborhoods without breaking existing all-active soft bodies. | Contact-local activation gives matching contact behavior with fewer active DOFs and deterministic state hashes. |
| WP-DB.06 CPU data layout and SIMD | Profile point-mass loops, identify contiguous/SoA/SIMD candidates, and use `dart/simd/` where it wins. | Benchmarks show single-core speedup without changing results beyond approved tolerances. Initial profiling, FCL soft-mesh refit reduction, parent-term scalar caching, vector-backed point-state/property reads in the bias-force edge loop, kinematics preflight, inverse-dynamics force updates, impulse/constrained-term state updates, matrix/vector aggregation state reads, the internal span-backed `PointMassPhaseView`, fused cached point-force aggregation, the articulated-inertia timing correction, conservative bias-force connection-loop cleanup, soft aggregation temporary cleanup, the `dart6-memory-hardening` stack, zero-allocation native soft-box plus soft-soft stack gates, and the heap-free `softBodies.skel`, `soft_open_chain`, and contact-producing `soft_cubes` SKEL gates are in `04-data-layout-and-memory-hardening.md`; heap-free contiguous point-mass object storage, retained SoA scratch, SIMD, and final idle-host threshold evidence remain open. |
| WP-DB.07 multi-core scaling | Parallelize independent soft-body loops or scene islands behind existing thread controls. | `threads=16` rows improve on representative scenes while `threads=1` remains deterministic. Pair-level native soft-soft worker scheduling exists; broader scaling evidence still needs larger scenes or intra-pair/active-neighborhood work. |
| WP-DB.08 native collision deformables | Make `dart/collision/native` the preferred collision solution for soft/deformable scenes instead of treating FCL as the destination backend. | Native soft-vs-plane, soft-vs-box, soft-vs-sphere, soft-vs-ellipsoid, and cached soft-vs-soft vertex-face contact lanes now have focused tests, with retained soft-face BVH pruning, pair-level finite-finite soft worker scheduling, primitive-frame soft-vs-primitive classification, direct `NativeCollisionDetector` soft/ellipsoid fallback coverage, native-detector `test_SoftDynamics` coverage, `drop_box` FCL/native checksum parity for the soft-box lane, warning-free native dispatch on the representative `soft_bodies` diagnostic, zero-allocation native soft-box plus soft-soft stack gates, and heap-free `softBodies.skel`, `soft_open_chain`, and contact-producing `soft_cubes` SKEL gates. Remaining acceptance requires fuller triangle/contact-neighborhood coverage, deterministic longer-run headless checksums, and threshold-quality performance rows where direct `native` matches or beats every checksum-equivalent backend on representative soft scenes. Current gap analysis is in `05-native-collision-deformable-lane.md`. |
| WP-DB.09 flagship demos | Add paper-parity basic and flagship examples that are runnable headlessly and, where useful, visually. | Each representative feature/performance/correctness claim has a demo or benchmark artifact. |

The paper-to-packet mapping lives in `02-paper-parity-matrix.md`.

## Verification

- Always run `pixi run lint` before commit.
- For benchmark harness changes: configure/build the benchmark target and run a
  short filtered benchmark such as:

  ```bash
  pixi run cmake --build build/default/cpp/Release --target BM_INTEGRATION_soft_body --parallel 8
  pixi run bm-soft-body --benchmark_filter=SoftBodyStep --benchmark_min_time=0.01s
  ```

- For dynamics changes: run `pixi run build`, focused soft-body integration
  tests, and the soft-body benchmark before/after on the same host.
- For contact or solver changes: add the Gazebo gate from `docs/ai/verification.md`
  when the touched surface can affect downstream collision/constraint behavior.

## Open decisions

- Whether DART 6 should preserve `SoftBodyNode` as the public API and replace
  internals, or introduce an opt-in compatibility-preserving deformable solver
  alongside it.
- Which external implementations are the formal "competitive implementations"
  for measured comparison on DART 6.
- Which paper demos are mandatory for the first release-branch PR versus later
  follow-up packets.
- Whether native soft collision should initially support full triangle meshes,
  adaptive active-vertex contact neighborhoods, or both. The long-term target is
  native as the best DART collision backend; the current FCL work is an interim
  measured fix for the existing soft-body path.
- The completing PR should include same-host baseline-vs-branch performance
  rows plus commands and locally captured videos for any newly added GUI
  deformable-body examples. Current smoke rows are in `06-pr-evidence.md`, but
  the final PR should still use a cleaner host run before setting threshold
  claims.
