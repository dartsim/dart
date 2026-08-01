# DART 6 deformable body feature and performance

Owner surface for restoring DART 6 deformable-body simulation quality without
using DART 7 clean-break APIs as the implementation target.

## Objective

Make DART 6 deformable-body simulation feature-complete, stable, and fast enough
to beat representative competing in-tree implementations and at least match the
published real-time or near-real-time CPU targets of **Jain and Liu 2011**.

**DART 6 scope is the Jain/Liu lane only.** Kim and Pollard's reduced volumetric
FEM was removed from `release-6.20` on 2026-07-29 and retargeted to DART 7,
because the two papers need different discretizations and a compatibility
release branch should carry one deformable model. Do not restart it here. The
durable owner of that scope is `docs/design/dart6_deformable_body.md`.

## Current milestone - soft-foot SIMBICON

The representative performance and soft-contact slice landed in
[#3382](https://github.com/dartsim/dart/pull/3382) on 2026-07-29 as
`6c88ac1d774a702b494643fb598be6b8af9385e1`. Its required hosted checks,
including Windows, Linux, macOS, FreeBSD, gz-physics, coverage, SIMD, API
documentation, and Read the Docs, completed successfully.

The DART-owned collision implementation then became the single built-in
`dart` detector through
[#3381](https://github.com/dartsim/dart/pull/3381). DART 6.20 keeps FCL as the
default. The Kim/Pollard volumetric-FEM subsystem was removed through
[#3407](https://github.com/dartsim/dart/pull/3407), leaving this task focused
on the Jain/Liu point-mass surface model.

[PR-3a soft-foot SIMBICON](12-pr3a-soft-foot-simbicon.md) is complete:
[#3408](https://github.com/dartsim/dart/pull/3408) added the scene and gates,
and [#3423](https://github.com/dartsim/dart/pull/3423) resolved the maintainer
decisions. Both Jain/Liu biped claims reproduce and are gate-asserted —
contacts 51.2 soft vs 15.64 rigid, recoverable push 18000 N soft vs 8000 N
rigid. The maintainer-set goal (2026-08-01) is to fully complete this task
for the DART 6.20 release, bundling the remaining work into as few PRs as
review quality allows.

Still open under PLAN-622: the motor-noise variant of the push-recovery
comparison, the noisy-floor biped and soft-contact walk rows, the hand/arm
manipulation rows (finger flick, arm fold, pinch grasp), the four-link
flexible-foot comparison, the competitive-envelope definition, WP-DB.07
multicore scaling, the pre-default `dart` detector coverage gates, a valid
`bm-soft-body-paired` artifact or approved disposition, and the separate
`main` fix for the zero-DoF soft point-mass assertion. The ordered list and
suggested PR bundles are in `RESUME.md`.

## Reference scope

**Active for DART 6:**

- Jain and Liu, "Controlling Physics-Based Characters Using Soft Contacts"
  (SIGGRAPH Asia 2011): point-mass surface flesh attached to articulated rigid
  bodies, vertex and edge springs, adaptive active surface vertices near
  contact, and LCP contact/friction coupling for controller robustness. This is
  the model `SoftBodyNode` implements.

**Not DART 6 work — retargeted to DART 7 on 2026-07-29:**

- Kim and Pollard, "Fast Simulation of Skeleton-driven Deformable Body
  Characters" (ACM TOG 30(5), 2011): reduced nonlinear volumetric FEM, two-way
  skeleton/deformable/environment coupling, explicit integration. Retained here
  only as the paper ledger. Whether a reduced FEM is still the right DART 7
  target is open given newer solvers such as AVBD.

Before this task, DART 6 `SoftBodyNode` resembled the Jain/Liu point-mass
surface model without proving the coupled equations, adaptive activation,
contact correctness, or a measured performance envelope. #3382 closes a
representative release slice of those gaps; the remaining **Jain/Liu** rows,
large-scene scaling, and pre-default collision contracts stay open below.

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
- `dart/dynamics/SoftBodyNode` still stores `std::vector<PointMass*>`.
  Contiguous object storage and retained mirrors were investigated and parked
  or rejected on design/measurement evidence; they are follow-up research, not
  unfinished edits to smuggle into #3382.
- `dart/constraint/SoftContactConstraint.*` supports soft-body contact plumbing
  through `SoftMeshShape` point selection and friction directions.
- `tests/integration/test_SoftDynamics.cpp` contains active finite-state,
  deterministic threaded final-state, DART-detector, and representative
  equations-of-motion gates for deformable scenes. WP-DB.04 now covers
  point-mass mass-matrix, augmented-mass, inverse-mass,
  inverse-augmented-mass, gravity, combined-vector, external-force, and
  representative rigid-only/soft-only/mixed-world matrix/vector checks,
  recorded in `07-equation-correctness.md`.
- `data/skel/test/test_double_pendulum.skel` used one legacy `<soft>` tag that
  `SkelParser` ignored; WP-DB.01 converts it to `<soft_shape>`, and WP-DB.02
  adds explicit soft-box fragments so both soft links parse cleanly.
- Legacy GUI examples remain in `examples/soft_bodies` and
  `examples/mixed_chain`; there was no headless soft-body benchmark before this
  task.
- `adaptive_soft_contact` and `soft_worm` are integrated scenes under
  `examples/demos`, hosted by `dart-demos`; their former standalone executable
  directories were removed. GUI-free model tests preserve their numerical
  contracts. New GUI examples in this lane should continue to be integrated
  into `dart-demos`, not added as standalone executables.
- WP-DB.01 now adds `BM_INTEGRATION_soft_body` and records smoke rows in
  `01-baseline-evidence.md`.
- WP-DB.03 now converts Kim/Pollard 2011 and Jain/Liu 2011 into a concrete
  parity ledger in `02-paper-parity-matrix.md`, including paper scene metrics,
  current DART 6 evidence, and acceptance gates. Representative demos and
  adaptive-contact work landed, while rows lacking a scene or an explicitly
  covered durable deferral remain open.
- WP-DB.06 groundwork adds `soft_body_headless`, a deterministic checksum and
  text-profiler runner for soft-body scenes. Initial profile evidence and the
  `origin/dart6-memory-hardening` dependency analysis are recorded in
  `04-data-layout-and-memory-hardening.md`.
- Commits `9a7bab76948` and `a122c5ab437` add and correct
  `bm-soft-body-paired`, a clean-HEAD evidence runner that checks
  supported detector rows at threads 1 and 16, preserves raw CPU-time rows and
  host-state history, alternates detector order across 20 pairs per row, and
  requires `COMPLETE.json` before any verdict is valid. No complete paired
  artifact exists yet; `verification.md` owns that evidence boundary.
- `docs/design/dart6_deformable_body.md` now owns the staged DART-owned
  soft-kernel follow-up contract, and PLAN-622 owns the separate `main`
  zero-DoF assertion fix. These facts no longer depend on this temporary task
  folder for survival.
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
  conservative connection-loop cleanup in `updateBiasForce()`. Historical
  comparisons are owned by merged PR #3382; no incomplete local run is a final
  detector ranking. A follow-up aggregation-temporary cleanup removes
  avoidable heap-temporary and no-op
  point-loop work from soft-body mass/gravity/force aggregation paths while
  preserving checksums and allocation gates. The branch incorporates the
  former `dart6-memory-hardening` stack's
  `World`/`MemoryManager`/`FrameAllocator` surfaces for DART soft allocation
  gates; that historical remote branch is not a current integration target.
  The latest local gates prove the measured `dart` soft-box post-bake steps,
  a two-soft-box `dart` stack steady-state window, and the
  `softBodies.skel`, `soft_open_chain.skel`, and contact-producing
  `soft_cubes.skel` SKEL-authored windows perform zero `operator new`, zero
  raw `malloc`, and zero base allocator growth. Contiguous point-mass storage
  and SIMD remain follow-up research under the measured dispositions in
  `04-data-layout-and-memory-hardening.md`.
- WP-DB.08 adds dynamic `SoftMeshShape` bounds and DART-owned point-mass
  contacts for soft-vs-plane, soft-vs-box, soft-vs-sphere, and
  soft-vs-ellipsoid pairs, plus a first soft-vs-soft vertex-face lane. The
  soft-soft lane now uses a backend-internal `DARTCollisionObject` soft mesh
  cache for local vertices, point-to-face metadata, and precomputed triangle
  geometry, early face-side separation rejection before barycentric projection,
  retained soft-face BVH traversal once a penetrating candidate exists, and
  pair-level worker scheduling in `DARTCollisionDetector` for eligible
  finite-finite soft pairs. The
  soft-vs-primitive lanes also reuse cached local vertices and representative
  face IDs. A follow-up primitive-frame slice classifies soft-vs-plane,
  soft-vs-sphere, and soft-vs-box point contacts in primitive-local
  coordinates and only computes world contact points for colliding vertices.
  A 200-step `drop_box` headless run matched FCL checksums exactly and ran
  faster with `COLLISION_DETECTOR=dart` on this host, and the broader
  `soft_bodies` diagnostic now runs without shape-creation or unsupported-pair
  warnings. Broader triangle/contact coverage and stronger multicore scaling
  beyond small pair counts remain open before proposing `dart` as the default.
  Allocation gates cover the soft-box contact lane, a soft-soft stack
  steady-state window, and the `softBodies.skel` no-contact soft-dynamics
  window, plus `soft_open_chain` and contact-heavy `soft_cubes` SKEL-authored
  windows. The durable follow-up contract lives in
  `docs/design/dart6_deformable_body.md`.

## Work packets

| Packet | Current disposition | Acceptance evidence |
| --- | --- | --- |
| WP-DB.01 baseline harness | Complete. | Headless benchmark rows cover representative soft scenes, point-mass/body counts, and thread settings (`01-baseline-evidence.md`). |
| WP-DB.02 stability gate | Merged in #3382. | Finite-state, thread-determinism, energy, contact-force/CoP smoothness, LCP robustness, and equation gates run in `test_SoftDynamics`; the final hosted Windows and full matrices passed (`03-stability-gate.md`, `07-equation-correctness.md`, `verification.md`). |
| WP-DB.03 paper parity matrix | Ledger complete; parity closeout still conditional. | Static paper targets now live in `docs/background/deformable_body_paper_targets.md`, and approved scope decisions live in `docs/design/dart6_deformable_body.md`. The four-link flexible-rigid-foot versus deformable-foot row is active DART 6 work and not deferred; the Kim/Pollard rows are out of DART 6 scope after being retargeted to DART 7 (`docs/design/dart6_deformable_body.md`). |
| WP-DB.04 coupled equation correctness | Merged in #3382. | Matrix/vector projection and inverse-identity gates plus the retained-acceleration independence regression pass (`07-equation-correctness.md`, `verification.md`). |
| WP-DB.05 adaptive contact activation | Complete. | Opt-in ABI-safe activation is default-off bit-identical, deterministic when enabled, allocation-gated, and covered by two recorded review rounds (`08-adaptive-contact-activation.md`). |
| WP-DB.06 CPU data layout and SIMD | #3382 disposition complete; follow-up research remains. | Kept cache/data-access slices produce the measured win; retained SoA mirrors and contiguous-object prototypes were rejected or parked because measurements/design gates did not justify keeping them. No unsupported SIMD speedup is claimed (`04-data-layout-and-memory-hardening.md`). |
| WP-DB.07 multi-core scaling | Original acceptance unmet; retained as an open PLAN-622 follow-up. | `DARTCollisionDetector` pair-level work and 1/4/16-thread determinism landed, but the tracked small scenes are flat or slower at 16 threads. The original `threads=16` improvement contract therefore remains open and needs a larger workload or a maintainer-approved negative disposition (`verification.md`, `docs/plans/dashboard.md`). |
| WP-DB.08 DART collision deformables | #3382 landing slice implemented; original acceptance unmet. | Primitive/cached soft lanes, face-interior coverage, allocation gates, and determinism landed. The `dart` detector is not yet the default, and broader coverage and scaling remain open under the durable contract in `docs/design/dart6_deformable_body.md` and PLAN-622. |
| WP-DB.09 flagship demos | Representative demos complete; parity closeout conditional. | The `dart-demos` scenes `adaptive_soft_contact` and `soft_worm` are runnable, and GUI-free model tests preserve their numerical contracts. The four-link flexible-rigid-foot versus deformable-foot comparison remains open and is active DART 6 work (`docs/design/dart6_deformable_body.md`, `verification.md`). |

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
- For the flagship scenes/models: build and run
  `test_AdaptiveSoftContactModel` and `test_SoftWormModel`; they are the
  current GUI-free finite-state, comparison, locomotion, and exact-repeat
  gates.

## Closeout decisions and follow-ups

- `SoftBodyNode` remains the DART 6 public API; implementation state stays out
  of public layouts. The completing PR uses additive non-virtual controls only.
- The 2026-07-11 paper-scale deferral list is **retracted**. Every Jain/Liu row
  is active DART 6 work, and the Kim/Pollard rows are out of DART 6 scope after
  being retargeted to DART 7 on 2026-07-29. The durable owner of that scope is
  `docs/design/dart6_deformable_body.md`; `decisions.md` holds the working
  record.
- The proposed formal competitive-implementation envelope is the in-tree
  CPU/backend comparison plus normalized paper metrics. It still needs
  maintainer sign-off before the broad objective or task retirement can be
  claimed; external-engine comparison remains a follow-up.
- The `dart` detector owns the current soft kernels. Broader coverage,
  scaling, and paired same-host evidence remain required before any default
  proposal.
- The zero-DoF soft point-mass assertion fix in `10c6b6055e4` also applies to
  `main` and requires the dual-PR follow-up. The current mass-matrix review fix
  is release-only because DART 7 still has point-mass mass aggregation disabled.
- Durable promotion has started in
  `docs/background/deformable_body_paper_targets.md`,
  `docs/design/dart6_deformable_body.md`, and PLAN-622. Before retiring this
  temporary task folder, complete or explicitly disposition the remaining
  competitive-envelope, flexible-foot, scaling, collision-coverage, paired
  benchmark, and cross-branch bug-fix work, then verify that no required fact
  remains owned only by this folder.
