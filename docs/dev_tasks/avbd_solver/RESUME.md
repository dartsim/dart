# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

The exact 176-row VBD/AVBD paper-parity contract remains fail-closed, with no
complete rows. The latest local packet advanced
`avbd.method.quasi_newton_hessian` from missing to partial: the Section 3.5
force-scaled column-norm diagonal now covers CPU deformable distance springs,
rigid distance springs, rigid point attachments, and nonlinear point-pair
joint/motor rows. Contact rows encode their intentional Taylor-linearized
zero-curvature policy. Angular/material families, the pinned 3D spring source
divergence, full source-corpus evidence, CUDA, and comparable paper/reference
performance remain open.

The durable packet is
[`../../plans/104-vertex-block-descent-solver/avbd-quasi-newton-evidence.json`](../../plans/104-vertex-block-descent-solver/avbd-quasi-newton-evidence.json).
It records 115 rigid and 9 deformable focused tests, full 6-DOF
finite-difference coverage, world-base/global/raw allocation closure, a fresh
uncached Release/profile build, 168/168 selected unit tests, exact-HEAD
interleaved mechanism benchmarks, a 60-second Spring text oracle, and an
assessed 120-frame software render. The method row stays partial.

## Current Branch

`feature/vbd-avbd-paper-parity-contract`, based on the then-current
`origin/main` at `83110ef54ab`. The fail-closed contract is committed locally
at `710cbfc1152`; the branch has not been pushed.

## Immediate Next Step

Implement `avbd.method.parallel_dual_stiffness_pass` next as a bounded
deterministic CPU packet:

- factor the post-primal dual/stiffness update over promoted deformable and
  rigid row inventories;
- prove serial/parallel state equivalence, row-order and thread-count
  determinism, bounds/fracture behavior, and non-finite failure propagation;
- add warmed-step world-base/global/raw allocation gates and matched throughput
  measurements;
- keep the canonical row partial until every paper row family and CUDA use the
  same contract.

## Context That Would Be Lost

- **Keep claims narrow.** Do not claim a source-demo/paper CPU win, GPU parity,
  a broad breakable-wall/fracture corpus, a same-hardware paper-number match, or
  an all-coefficient friction win unless the tracked artifacts directly prove it.
- The pinned 2D source applies Section 3.5 generically and the 3D joint source
  applies it to nonlinear point rows, but the pinned 3D Spring source omits the
  geometric term. DART follows the paper for distance springs; do not call that
  source equivalence without explicit adjudication and evidence.
- The durable completion rule is
  [`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md);
  the older corpus matrix's `Complete` labels describe narrow historical assets,
  not canonical parity rows.
- Two allocator tests previously recorded as profile-build failures
  (`DantzigSolver.ScratchUsesProvidedAllocatorForDantzigWorkBuffers` and
  `World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator`) both pass in the
  current fresh uncached profile build; do not carry the old failure claim
  forward without a new reproduction.
- PLAN-104's progress log + git history hold the full per-slice record; this
  folder no longer duplicates it.

## How to Resume

```bash
git status --short --branch
git log --oneline --decorate -5
pixi run check-plan104-paper-parity
pixi run check-avbd-packets
```

Stay in this exact worktree and branch. Start from PLAN-104's first current gap,
the Section 4 parallel dual/stiffness pass. Verify with focused AVBD and world
allocation tests, matched serial/parallel state evidence, the contract checker,
and `pixi run lint`; do not push or mutate GitHub without explicit approval.
