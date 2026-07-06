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
- `tests/integration/test_SoftDynamics.cpp` contains an equations-of-motion
  comparison that remains disabled because soft-body matrix aggregation is not
  complete. WP-DB.02 adds an active finite-state gate around representative
  soft scenes.
- `data/skel/test/test_double_pendulum.skel` used one legacy `<soft>` tag that
  `SkelParser` ignored; WP-DB.01 converts it to `<soft_shape>`, and WP-DB.02
  adds explicit soft-box fragments so both soft links parse cleanly.
- GUI examples exist in `examples/soft_bodies` and `examples/mixed_chain`, but
  there was no headless soft-body benchmark before this task.
- WP-DB.01 now adds `BM_INTEGRATION_soft_body` and records smoke rows in
  `01-baseline-evidence.md`.
- WP-DB.06 groundwork adds `soft_body_headless`, a deterministic checksum and
  text-profiler runner for soft-body scenes. Initial profile evidence and the
  `origin/dart6-memory-hardening` dependency analysis are recorded in
  `04-data-layout-and-memory-hardening.md`.
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
  `updateTransform`, `updateVelocity`, and `updatePartialAcceleration`. Current
  checksums stay stable, but timing remains too noisy for a hard speedup
  threshold. A narrow `origin/dart6-memory-hardening` carryover now scans
  body-local external wrenches directly during rest detection to avoid dirty
  projection-cache materialization. Contiguous point-mass storage and SIMD
  remain open.
- WP-DB.08 native collision slices add dynamic `SoftMeshShape` bounds and native
  point-mass contacts for soft-vs-plane, soft-vs-box, soft-vs-sphere, and
  soft-vs-ellipsoid pairs, plus a first soft-vs-soft vertex-face lane. The
  soft-soft lane now uses a backend-internal `DARTCollisionObject` soft mesh
  cache for local vertices, point-to-face metadata, and precomputed triangle
  geometry, early face-side separation rejection before barycentric projection,
  retained soft-face BVH traversal once a penetrating candidate exists, and
  pair-level worker scheduling for eligible finite-finite soft pairs. The
  soft-vs-primitive lanes also reuse cached local vertices and representative
  face IDs. A 200-step `drop_box` headless run matched FCL checksums exactly
  and ran faster with `COLLISION_DETECTOR=dart` on this host, and the broader
  `soft_bodies` native diagnostic now runs without shape-creation or
  unsupported-pair warnings.
  Broader scenes still need fuller triangle/contact-neighborhood coverage,
  stronger multicore scaling beyond small pair counts, and allocation gates
  before native can replace FCL.

## Work packets

| Packet | Scope | Done when |
| --- | --- | --- |
| WP-DB.01 baseline harness | Add headless Google Benchmark rows for existing soft-body scenes across single-core and multi-core CPU. | Benchmark target builds; rows report sim seconds per second, point masses, soft bodies, and threads. |
| WP-DB.02 stability gate | Re-enable or replace `test_SoftDynamics` with finite-state and equations-of-motion checks for representative soft scenes. | Focused integration test fails on NaN/Inf, gross state blow-up, or known equation regressions. Initial finite-state coverage is recorded in `03-stability-gate.md`; equation checks remain open. |
| WP-DB.03 paper parity matrix | Convert both papers into a scene/feature/performance checklist. | Every representative paper demo has an owner row, current DART 6 status, target evidence, and acceptance gate. |
| WP-DB.04 coupled equation correctness | Finish or replace stubbed point-mass mass, inverse-mass, gravity, combined-force, and external-force aggregation paths. | Matrix/vector checks pass for rigid-only, soft-only, and mixed rigid-soft worlds. |
| WP-DB.05 adaptive contact activation | Implement Jain/Liu-style active vertex neighborhoods without breaking existing all-active soft bodies. | Contact-local activation gives matching contact behavior with fewer active DOFs and deterministic state hashes. |
| WP-DB.06 CPU data layout and SIMD | Profile point-mass loops, identify contiguous/SoA/SIMD candidates, and use `dart/simd/` where it wins. | Benchmarks show single-core speedup without changing results beyond approved tolerances. Initial profiling, FCL soft-mesh refit reduction, parent-term scalar caching, vector-backed point-state/property reads in the bias-force edge loop and kinematics preflight, a small rest-detection memory-hardening carryover, and memory-hardening dependency notes are in `04-data-layout-and-memory-hardening.md`; heap-free contiguous storage and SIMD remain open. |
| WP-DB.07 multi-core scaling | Parallelize independent soft-body loops or scene islands behind existing thread controls. | `threads=16` rows improve on representative scenes while `threads=1` remains deterministic. Pair-level native soft-soft worker scheduling exists; broader scaling evidence still needs larger scenes or intra-pair/active-neighborhood work. |
| WP-DB.08 native collision deformables | Make `dart/collision/native` the preferred collision solution for soft/deformable scenes instead of treating FCL as the destination backend. | Native soft-vs-plane, soft-vs-box, soft-vs-sphere, soft-vs-ellipsoid, and cached soft-vs-soft vertex-face contact lanes now have focused tests, with retained soft-face BVH pruning, pair-level finite-finite soft worker scheduling, `drop_box` FCL/native checksum parity for the soft-box lane, and warning-free native dispatch on the representative `soft_bodies` diagnostic. Remaining acceptance requires fuller triangle/contact-neighborhood coverage, deterministic longer-run headless checksums, and performance rows that match or beat FCL on representative soft scenes. Current gap analysis is in `05-native-collision-deformable-lane.md`. |
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
  deformable-body examples.
