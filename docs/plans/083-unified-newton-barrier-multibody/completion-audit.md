# PLAN-083 Completion Audit

Audit date: 2026-06-10

Verdict: NOT COMPLETE.

PLAN-083 has completed the implementation-roadmap Phase 8 audit step and now has
branch-local runtime smoke coverage for several landed Newton-barrier contracts,
but the plan itself cannot be called complete yet. The manifest rows are all
classified, but many remain in-progress with reduced evidence and explicit
limitations. The CPU scene corpus plus private GPU parity packet also still
carry explicit limitations. Do not retire
`docs/dev_tasks/unified_newton_barrier_multibody/`
until a maintainer decides whether the remaining work stays tracked there or
moves entirely into durable plan sidecars.

## Evidence Snapshot

| Artifact                 | Row count | Status counts                          |
| ------------------------ | --------- | -------------------------------------- |
| `paper-deck-manifest.md` | 48        | `in-progress`: 48                      |
| `cpu-scene-corpus.json`  | 27        | `in-progress`: 26, `not-applicable`: 1 |
| `gpu-parity-packet.json` | 6         | `in-progress`: 5, `measured`: 1        |

## Phase 8 Checklist

| Requirement                                                   | Audit result | Notes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ------------------------------------------------------------- | ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `paper-deck-manifest.md` has zero unclassified rows           | Pass         | Every manifest row has an explicit status. The current manifest rows are all `in-progress`; none are unclassified or complete.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| Every non-planned row links to concrete evidence or rationale | Partial      | In-progress rows name branch-local artifacts, tests, packets, or limitations. Runtime wiring now covers point/hinge constraints, BDF-2, deformable surface obstacles, reduced lying-flat, hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll, nunchaku, nunchaku scaling, windmill, Candy, precession, reduced timing-breakdown packets, reduced Table 2 setup/statistics packets, the sparse equality change-of-variable rigid IPC path, reduced ABD house-of-cards and wrecking-ball runtime-step packets, reduced ABD chain-net runtime-step packets, reduced ABD gears/Bullet/complex-geometry runtime-step packets, a reduced ABD/FEM coupled micro-solve packet, built-in deformable `World::step` self-surface candidate/CCD counters in the reduced lying-flat, Candy, and ABD/FEM CPU scene packets, public built-in external surface CCD diagnostics for inter-body deformable, static rigid, and moving rigid obstacles, reduced CPU packet serialization for those external counters, a dedicated reduced external surface CCD CPU packet with nonzero inter-body/static-rigid/moving-rigid counter evidence plus one mixed reduced `World::step` witness, reduced lying-flat CPU packet inter-body/static-rigid/moving-rigid surface CCD witnesses, and a reduced Candy CPU packet static-rigid/moving-rigid surface CCD witness pair, but not paper-scale behavior or performance parity.                                                                                                                                                                                                                                                                                                                              |
| CPU packets exist for performance rows                        | Partial      | `cpu-scene-corpus.json` records commands and artifact paths for every row, and the reduced deformable `World::step` rows now carry self-surface candidate/CCD counters plus serialized external inter-body/static-rigid/moving-rigid surface CCD counters. The dedicated `unb-alg-barriers` external surface CCD diagnostic packet now exercises nonzero external counters in isolated witnesses and in one mixed reduced `World::step` scene that activates all three external families; the reduced `unb-fig-01` lying-flat benchmark row now carries nonzero inter-body, static-rigid, and moving-rigid surface CCD witnesses, and the reduced `unb-fig-22` Candy benchmark row now carries nonzero static-rigid and moving-rigid surface CCD witnesses. Other broader figure/demo scene rows still have zero external candidate/check/hit counts, so production paper-scene external contact remains unproven. Many entries still remain reduced packets or placeholders rather than paper-scale reproductions, accepted Bullet/reference comparisons, or GPU parity evidence.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| GPU packets exist for GPU claims                              | Blocked      | `gpu-parity-packet.json` has a measured PSD projection packet plus in-progress point-triangle/edge-edge contact-stencil, endpoint-linear point-triangle and edge-edge CCD/line-search, scalar barrier/friction local-kernel plus point-triangle primitive barrier-gradient, all four primitive-family tangent-stencil parity packets, point-triangle/point-point/point-edge/edge-edge primitive barrier-Hessian parity, point-triangle/point-point/point-edge/edge-edge primitive barrier-Hessian PSD-projection parity, reduced scene-owned point-triangle/point-edge/point-point/edge-edge barrier-Hessian runtime rows plus a reduced combined all-family scene runtime barrier-Hessian row, reduced combined scene runtime candidate-filter evidence, reduced diagonal assembly/solve plus reduced scene-owned diagonal assembly/solve plus pair-slot off-diagonal sparse-block assembly plus reduced scene-owned sparse off-diagonal surface-edge assembly plus reduced scene-owned sparse graph construction/assembly and unique-edge dedup rows plus reduced scene-owned nonlinear distance-equality assembly, one-step solve, and capped convergence rows plus sparse block residual matvec plus reduced scene-owned sparse residual matvec plus fixed-iteration sparse Jacobi solve plus reduced scene-owned sparse Jacobi solve plus capped sparse CG solve plus reduced scene-owned sparse CG solve plus bounded reduced direct sparse factor solve plus reduced scene-owned bounded direct sparse factor solve plus sparse equality-reduced diagonal solve plus reduced scene-owned equality-reduced diagonal solve, and reduced scene state-batch parity packets, but no full scene-level GPU `World::step` speed claim exists. |
| Public headers and dartpy remain backend-neutral              | Pass         | Phase 8 validation keeps API-boundary gates in the local evidence set; no public CUDA/backend/solver-registry API is added by this audit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| Durable docs own landed architecture                          | Partial      | Durable plan sidecars own the manifest, CPU corpus, GPU packet, and this audit. The temporary dev-task folder remains active because acceptance is unmet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| Temporary dev-task folder retired                             | Blocked      | Retirement requires maintainer direction because material in-progress work remains.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

## Remaining Blockers

- Runtime mixed-domain stepping and sparse equality-constraint solving are only
  partially promoted: rigid IPC now covers point/fixed and hinge constraints,
  opt-in BDF-2, deformable surface obstacles, reduced lying-flat/Candy
  self-surface contact candidate and CCD diagnostics, a reduced Candy
  static-rigid/moving-rigid surface CCD witness pair, public built-in external
  surface CCD diagnostics, a dedicated reduced external surface CCD diagnostic
  CPU packet with one mixed reduced `World::step` witness,
  hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll, nunchaku,
  nunchaku scaling, windmill, Candy, precession, reduced timing-breakdown,
  reduced Table 2 setup/statistics packets, the sparse equality
  change-of-variable rigid IPC path, reduced ABD house-of-cards and
  wrecking-ball runtime-step packets, reduced ABD chain-net runtime-step
  packets, reduced ABD gears/Bullet/complex-geometry comparison runtime-step
  packets, and a reduced ABD/FEM coupled micro-solve packet, but paper-scale mixed
  scenes plus analytical pulley/sliding reproductions remain unproven.
- CPU benchmark/profile packets do not yet reproduce every paper/deck
  performance row against DART incumbents, audited references, and paper/deck
  numbers. Reduced ABD gears/Bullet/complex-geometry packets and a reduced
  ABD/FEM coupled micro-solve packet now exist, and the deformable reduced
  scene packets now carry built-in self-surface candidate/CCD counters and
  serialized external inter-body/static-rigid/moving-rigid surface CCD counters.
  A dedicated external surface CCD diagnostic packet now has nonzero
  inter-body/static-rigid/moving-rigid counter evidence in aggregate
  (66/136/152 hits on the latest median row) and in one mixed reduced
  `World::step` scene (33/68/76 hits). The reduced lying-flat figure row now
  carries one inter-body surface CCD witness (33 hits and one limited step),
  one static-rigid surface CCD witness (34 hits and one limited step), and one
  moving-rigid surface CCD witness (1 hit and one limited step). The reduced
  Candy figure row now carries one static-rigid surface CCD witness (72 hits
  and one limited step) and one moving-rigid surface CCD witness (184 hits and
  one limited step), but other broader figure/demo scene rows still have
  zero external candidate/check/hit counts; accepted
  Bullet/reference baselines, full runtime affine/FEM coupling, and paper-scale
  assets remain unproven.
- Private GPU packets do not yet prove same-scene CPU/GPU speedups for contact
  candidates, full CCD/line search, full local barrier/friction kernels, full
  sparse assembly/linear solve, or full scene-level workloads. The PSD
  projection local-kernel row is
  measured separately. The contact-stencil filter packet is in-progress because
  parity exists for preassembled point-triangle and edge-edge stencils plus
  brute-force all-pairs point-triangle and edge-edge candidate masks with
  device-side compacted candidate ids and distance metadata, motion-aware
  swept-AABB point-triangle/edge-edge candidate-list packets, device-sorted
  sweep-and-prune broad-phase packets, and compact runtime sweep-buffer
  endpoint-distance packets that consume CPU sweep candidate keys plus reduced
  scene-owned runtime candidate buffers, scene-owned runtime sweep broad-phase
  rows, a reduced combined scene runtime sweep-filter row, and a reduced
  combined scene runtime candidate-filter row extracted from one DART `World`
  deformable surface, but production runtime scene filtering inside
  `World::step`, GPU `World::step` contact candidate construction, and speedup
  remain unproven.
  The endpoint-linear point-triangle/edge-edge plus sampled rigid-curved
  point-triangle/edge-edge CCD/line-search packet is in-progress because
  parity exists for reduced endpoint-linear, 8-sample trajectory, and reduced
  scene-owned runtime candidate fixtures plus a reduced combined scene runtime
  CCD line-search row, but analytic curved CCD, production scene-level
  line-search feasibility inside `World::step`, and runtime speedup remain
  unproven. The barrier/friction
  local-kernel
  packet is
  in-progress because parity exists for clamped-log barrier derivatives,
  smoothed friction norm/work, point-triangle primitive barrier gradients,
  point-triangle, edge-edge, point-edge, and point-point tangent stencils,
  point-triangle/point-point/point-edge/edge-edge primitive barrier-Hessian rows, and
  point-triangle/point-point/point-edge/edge-edge primitive barrier-Hessian PSD
  projection, plus reduced scene-owned point-triangle/point-edge/point-point/
  edge-edge barrier-Hessian runtime rows plus a reduced combined all-family
  scene runtime barrier-Hessian row extracted from one DART `World` deformable
  surface, but broader sparse Hessian assembly, full GPU `World::step`, and the
  top-level/runtime speedup gate remain unproven.
  The reduced assembly/solve
  packet is in-progress because parity exists for diagonal per-body row
  assembly, a reduced scene-owned diagonal assembly/solve row extracted from
  one DART `World` deformable surface, independent regularized Newton steps,
  preallocated 6x6 off-diagonal pair slots, reduced scene-owned sparse
  off-diagonal surface-edge assembly rows, reduced scene-owned sparse graph
  construction/assembly rows, a reduced scene-owned sparse graph unique-edge
  dedup row, one symmetric sparse residual matvec, a reduced
  scene-owned sparse residual matvec, reduced scene-owned nonlinear
  distance-equality assembly rows, one reduced scene-owned nonlinear
  distance-equality Jacobi-style correction step, a reduced scene-owned capped
  distance-equality convergence row with endpoint residual reporting,
  fixed-iteration sparse Jacobi solves, a reduced scene-owned sparse Jacobi
  solve, capped sparse CG solves, a reduced scene-owned sparse CG solve, bounded
  direct sparse factor solves, a reduced scene-owned bounded direct sparse
  factor solve, sparse equality-reduced diagonal solves, and a reduced
  scene-owned equality-reduced diagonal solve only, but full
  runtime sparse Hessian graph construction and assembly beyond the reduced
  dedup row, unbounded production direct/global sparse factorization,
  production nonlinear equality convergence policy/solving, GPU `World::step`
  assembly/solve integration, and speedup remain unproven.
  The reduced
  scene parity/speed packet is in-progress because
  parity and speedup exist only for DART scene state extraction plus private
  rigid-body state-batch CUDA rollout; GPU `World::step`, contact candidate
  construction, CCD, barrier/friction assembly, sparse equality reduction, and
  global Newton solving remain unproven.
- Launchable py-demo placeholders exist, and the reduced lying-flat,
  hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll, nunchaku,
  windmill, Candy, and precession scenes have local headless smoke/capture
  evidence, but the remaining
  paper-scale scenes still need long-horizon visual captures and nonblank/motion
  evidence.
- ABD remains an internal primitive/oracle plus reduced point-triangle
  micro-solve, runtime-step, pair-runtime, chain-net, comparison, and ABD/FEM
  coupled micro-solve packets with residual/convergence counters, not a
  paper-scale runtime affine solver or a replacement for the rigid IPC
  correctness oracle.

## Maintainer Decision Required

Choose one of these before closing the active dev task:

1. Keep `docs/dev_tasks/unified_newton_barrier_multibody/` active for the
   remaining PLAN-083 implementation work.
2. Retire the dev-task folder in a later PR only after relocating every remaining
   blocker above into durable plan/dashboard ownership and accepting that
   PLAN-083 is still incomplete.
