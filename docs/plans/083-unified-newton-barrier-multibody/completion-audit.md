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

| Artifact                 | Row count | Status counts                                         |
| ------------------------ | --------- | ----------------------------------------------------- |
| `paper-deck-manifest.md` | 48        | `in-progress`: 26, `planned`: 22                      |
| `cpu-scene-corpus.json`  | 26        | `in-progress`: 15, `not-applicable`: 1, `planned`: 10 |
| `gpu-parity-packet.json` | 6         | `measured`: 1, `planned`: 5                           |

## Phase 8 Checklist

| Requirement                                                   | Audit result | Notes                                                                                                                                                                                                                                                                                                                                        |
| ------------------------------------------------------------- | ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `paper-deck-manifest.md` has zero unclassified rows           | Pass         | Every manifest row has an explicit status. The current statuses are `planned` or `in-progress`; none are unclassified.                                                                                                                                                                                                                       |
| Every non-planned row links to concrete evidence or rationale | Partial      | In-progress rows name branch-local artifacts, tests, packets, or limitations. Runtime wiring now covers point/hinge constraints, BDF-2, deformable surface obstacles, reduced hanging-bridge, terrain vehicle, nunchaku, windmill, and precession scenes, and their reduced CPU packets, but not paper-scale behavior or performance parity. |
| CPU packets exist for performance rows                        | Blocked      | `cpu-scene-corpus.json` records commands and artifact paths, but 10 performance/comparison rows remain `planned` and many others are placeholders.                                                                                                                                                                                           |
| GPU packets exist for GPU claims                              | Blocked      | `gpu-parity-packet.json` has a measured PSD projection packet, but five rows remain `planned` and no scene-level GPU speed claim exists.                                                                                                                                                                                                     |
| Public headers and dartpy remain backend-neutral              | Pass         | Phase 8 validation keeps API-boundary gates in the local evidence set; no public CUDA/backend/solver-registry API is added by this audit.                                                                                                                                                                                                    |
| Durable docs own landed architecture                          | Partial      | Durable plan sidecars own the manifest, CPU corpus, GPU packet, and this audit. The temporary dev-task folder remains active because acceptance is unmet.                                                                                                                                                                                    |
| Temporary dev-task folder retired                             | Blocked      | Retirement requires maintainer direction because material planned work remains.                                                                                                                                                                                                                                                              |

## Remaining Blockers

- Runtime mixed-domain stepping and sparse equality-constraint solving are only
  partially promoted: rigid IPC now covers point/fixed and hinge constraints,
  opt-in BDF-2, deformable surface obstacles, and reduced hanging-bridge,
  terrain vehicle, nunchaku, windmill, and precession smoke scenes plus reduced CPU
  packets, but paper-scale mixed scenes remain unproven.
- CPU benchmark/profile packets do not yet reproduce or explain every paper/deck
  performance row against DART incumbents, audited references, and paper/deck
  numbers.
- Private GPU packets do not yet prove same-scene CPU/GPU parity or speedups for
  contact candidates, CCD/line search, local barrier/friction kernels, assembly,
  linear solve, or scene-level workloads. The PSD projection local-kernel row is
  measured separately.
- Launchable py-demo placeholders exist, and the reduced hanging-bridge,
  terrain vehicle, nunchaku, windmill, and precession scenes have local headless
  smoke/capture evidence, but the remaining paper-scale scenes still need
  long-horizon visual captures and nonblank/motion evidence.
- ABD remains an internal primitive/oracle micro-packet, not a runtime affine
  solver or a replacement for the rigid IPC correctness oracle.

## Maintainer Decision Required

Choose one of these before closing the active dev task:

1. Keep `docs/dev_tasks/unified_newton_barrier_multibody/` active for the
   remaining PLAN-083 implementation work.
2. Retire the dev-task folder in a later PR only after relocating every remaining
   blocker above into durable plan/dashboard ownership and accepting that
   PLAN-083 is still incomplete.
