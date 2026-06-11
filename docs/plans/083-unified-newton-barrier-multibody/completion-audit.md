# PLAN-083 Completion Audit

Audit date: 2026-06-10

Verdict: NOT COMPLETE.

PLAN-083 has completed the implementation-roadmap Phase 8 audit step and now has
branch-local runtime smoke coverage for several landed Newton-barrier contracts,
but the plan itself cannot be called complete yet. The manifest, CPU scene
corpus, and private GPU parity packet still contain planned rows and explicit
limitations. Do not retire `docs/dev_tasks/unified_newton_barrier_multibody/`
until a maintainer decides whether the remaining work stays tracked there or
moves entirely into durable plan sidecars.

## Evidence Snapshot

| Artifact                 | Row count | Status counts                                 |
| ------------------------ | --------- | --------------------------------------------- |
| `paper-deck-manifest.md` | 48        | `in-progress`: 45, `planned`: 3               |
| `cpu-scene-corpus.json`  | 26        | `in-progress`: 25, `not-applicable`: 1        |
| `gpu-parity-packet.json` | 6         | `in-progress`: 1, `measured`: 1, `planned`: 4 |

## Phase 8 Checklist

| Requirement                                                   | Audit result | Notes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ------------------------------------------------------------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `paper-deck-manifest.md` has zero unclassified rows           | Pass         | Every manifest row has an explicit status. The current statuses are `planned` or `in-progress`; none are unclassified.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| Every non-planned row links to concrete evidence or rationale | Partial      | In-progress rows name branch-local artifacts, tests, packets, or limitations. Runtime wiring now covers point/hinge constraints, BDF-2, deformable surface obstacles, reduced lying-flat, hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll, nunchaku, nunchaku scaling, windmill, Candy, precession, reduced timing-breakdown packets, reduced Table 2 setup/statistics packets, the sparse equality change-of-variable rigid IPC path, reduced ABD house-of-cards and wrecking-ball runtime-step packets, reduced ABD chain-net runtime-step packets, and reduced ABD gears/Bullet comparison runtime-step packets, but not paper-scale behavior or performance parity. |
| CPU packets exist for performance rows                        | Partial      | `cpu-scene-corpus.json` records commands and artifact paths for every row, but many entries remain reduced packets or placeholders rather than paper-scale reproductions, accepted Bullet/reference comparisons, or GPU parity evidence.                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| GPU packets exist for GPU claims                              | Blocked      | `gpu-parity-packet.json` has a measured PSD projection packet and an in-progress point-triangle contact-stencil filter parity packet, but four rows remain `planned` and no scene-level GPU speed claim exists.                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| Public headers and dartpy remain backend-neutral              | Pass         | Phase 8 validation keeps API-boundary gates in the local evidence set; no public CUDA/backend/solver-registry API is added by this audit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| Durable docs own landed architecture                          | Partial      | Durable plan sidecars own the manifest, CPU corpus, GPU packet, and this audit. The temporary dev-task folder remains active because acceptance is unmet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| Temporary dev-task folder retired                             | Blocked      | Retirement requires maintainer direction because material planned work remains.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

## Remaining Blockers

- Runtime mixed-domain stepping and sparse equality-constraint solving are only
  partially promoted: rigid IPC now covers point/fixed and hinge constraints,
  opt-in BDF-2, deformable surface obstacles, and reduced lying-flat,
  hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll, nunchaku,
  nunchaku scaling, windmill, Candy, precession, reduced timing-breakdown,
  reduced Table 2 setup/statistics packets, the sparse equality
  change-of-variable rigid IPC path, reduced ABD house-of-cards and
  wrecking-ball runtime-step packets, reduced ABD chain-net runtime-step
  packets, and reduced ABD gears/Bullet comparison runtime-step packets, but
  paper-scale mixed scenes plus analytical pulley/sliding reproductions remain
  unproven.
- CPU benchmark/profile packets do not yet reproduce every paper/deck
  performance row against DART incumbents, audited references, and paper/deck
  numbers. Reduced ABD gears/Bullet packets now exist, but accepted
  Bullet/reference baselines and paper-scale assets remain unproven.
- Private GPU packets do not yet prove same-scene CPU/GPU speedups for contact
  candidates, CCD/line search, local barrier/friction kernels, assembly, linear
  solve, or scene-level workloads. The PSD projection local-kernel row is
  measured separately, and the point-triangle contact-stencil filter packet is
  in-progress because parity exists for preassembled stencils but broad-phase,
  edge-edge, runtime scene filtering, and speedup remain unproven.
- Launchable py-demo placeholders exist, and the reduced lying-flat,
  hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll, nunchaku,
  windmill, Candy, and precession scenes have local headless smoke/capture
  evidence, but the remaining
  paper-scale scenes still need long-horizon visual captures and nonblank/motion
  evidence.
- ABD remains an internal primitive/oracle plus reduced point-triangle
  micro-solve, runtime-step, pair-runtime, chain-net, and comparison packets
  with residual/convergence counters, not a paper-scale runtime affine solver or
  a replacement for the rigid IPC correctness oracle.

## Maintainer Decision Required

Choose one of these before closing the active dev task:

1. Keep `docs/dev_tasks/unified_newton_barrier_multibody/` active for the
   remaining PLAN-083 implementation work.
2. Retire the dev-task folder in a later PR only after relocating every remaining
   blocker above into durable plan/dashboard ownership and accepting that
   PLAN-083 is still incomplete.
