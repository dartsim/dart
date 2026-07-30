# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

The exact 176-row VBD/AVBD paper-parity contract remains fail-closed, with no
complete rows. The latest local packet advanced
`avbd.method.parallel_dual_stiffness_pass` from missing to partial: the
Section 4 post-primal update now uses deterministic persistent-worker ranges
for sufficiently large promoted deformable and private free-rigid CPU row
inventories. The serial primal/color order remains unchanged. Articulated and
unified row coverage, CUDA, and comparable source-matched achieved-accuracy
performance remain open.

The durable packet is
[`../../plans/104-vertex-block-descent-solver/avbd-parallel-dual-update-evidence.json`](../../plans/104-vertex-block-descent-solver/avbd-parallel-dual-update-evidence.json).
It records bitwise serial/2-worker/4-worker coverage for every promoted row
kind, invalid and non-finite behavior, executor nesting/concurrency/exception
recovery, production-World activation, warmed world-base/global/raw allocation
closure, and the exact-parent interleaved throughput benchmark. The method row
stays partial.

## Current Branch

`feature/vbd-avbd-paper-parity-contract`, based on the then-current
`origin/main` at `83110ef54ab`. The fail-closed contract is committed locally
at `710cbfc1152`, and the Section 3.5 packet at `a78f688a178`; the branch has
not been pushed.

## Immediate Next Step

Advance PLAN-104's articulated multibody AVBD extraction gap. Preserve the
Section 4 claim boundary: the new CPU pass is only partial until articulated
and unified row inventories plus CUDA use the same contract and
source-matched achieved-accuracy evidence closes.

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
articulated multibody AVBD extraction. Verify with focused AVBD and world
allocation tests, matched solver-state evidence, the contract checker, and
`pixi run lint`; do not push or mutate GitHub without explicit approval.
