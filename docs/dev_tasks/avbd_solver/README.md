# AVBD Solver - Dev Task

Implementation tracking for PLAN-104's Augmented Vertex Block Descent work.
This folder is the temporary working surface; the durable owner is the plan.

- Plan: [`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md)
  (owns `Current Implementation Evidence`, `AVBD Current Next Gaps`,
  `Acceptance Criteria`, and the `Progress log`).
- Paper audit:
  [`../../plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](../../plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md).
- Corpus matrix:
  [`../../plans/104-vertex-block-descent-solver/avbd-demo-corpus.md`](../../plans/104-vertex-block-descent-solver/avbd-demo-corpus.md).

## Current Status

- **Active, incomplete paper implementation.** No parity is claimed. The
  headline gates remain **open**: a source-demo/paper CPU win, GPU parity, and a
  same-hardware paper-number match.
- **Branch state:** no active local AVBD branch; continue from current `main`.
- **Recent slices merged to `main`** (see the PLAN-104 progress log and the PRs
  for detail; per-slice history lives in git, not in this file):
  - #2991 — source-row coverage + contact-precheck (`f6fecbc5bd5`).
  - #3004 — 2D/3D Spring/Spring Ratio contact-filtering, inertia-orientation
    cleanup, refreshed packets, contact-skip regressions (`356384967f8`).
  - #3018 — box edge/vertex rigid-contact feature-ID coverage (`6bf7b2e8336`).
  - #3022 — bounded regression coverage: rigid-contact tangent-basis contract,
    articulated break→reset→break re-arm lifecycle, row-inventory replaced-key
    cold-start (`65ba05113c6`).
- **Out-of-scope pre-existing gates:** two allocator regressions
  (`DantzigSolver.ScratchUsesProvidedAllocatorForDantzigWorkBuffers`,
  `World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator`) are LCP/rigid-IPC
  issues, not AVBD. They reproduce only under `DART_BUILD_PROFILE=ON`; hosted CI
  is green. Recommended for a separate LCP/IPC allocator fix.

## Goal

Implement AVBD as the hard-constraint continuation of DART's VBD solver family:
all paper/reference algorithms and features, CPU and GPU parity, complete
paper/site/video/demo reproduction in DART tests/benchmarks/`py-demos`, and
performance that beats the reference demo repositories and published paper
numbers.

## Non-Goals For Early Phases

- Do not expose AVBD, row storage, solver registries, CUDA types, or ECS details
  as public API.
- Do not vendor or runtime-link the AVBD demo repositories.
- Do not claim AVBD parity from the scalar row utility, a CPU-only slice, or one
  demo scene.

## Key Decisions

- **Reuse PLAN-104:** AVBD extends the VBD solver family, so the durable owner is
  PLAN-104 plus its AVBD paper gap audit rather than a duplicate plan.
- **Row foundation first:** The scalar row update equations are shared by hard
  contact, joints, attachments, friction limits, motors, fracture, and
  finite-stiffness ramping, so they are the first tested implementation slice.
- **Clean DART 7/8 architecture:** AVBD work may refactor internal solver,
  pipeline, row-storage, compute, and demo surfaces when that produces a cleaner
  long-term design.

## Immediate Next Steps

Pick the next bounded, mutation-verified slice from PLAN-104's
`AVBD Current Next Gaps` and the corpus matrix; keep claims narrow (no CPU-win,
GPU, broad fracture-corpus, or same-hardware paper-number claim unless the
tracked artifacts prove it). Two small deferred items are also ready:

1. **Hoist `makeCollisionPairKey`** into a shared `detail` header. It is
   currently duplicated verbatim in `dart/simulation/compute/rigid_body_contact_stage.cpp`
   and `dart/simulation/world.cpp`, so the contact-stage ignored-pair audit and
   `World::setCollisionPairIgnored` can drift out of sync.
2. **Upgrade the Spring / Spring Ratio packets** from legacy `schema_version` 1
   (`LEGACY_IDENTITY_EXEMPT`) to version 2 with a `resolved_solver_identity` so
   the packet-identity check covers them.

## History

Per-slice history and durable evidence live in the PLAN-104 progress log, the
paper gap audit, the corpus matrix, git history, and the merged PRs above. Do
not re-accrete a session-by-session log in this file; keep it to current state.
