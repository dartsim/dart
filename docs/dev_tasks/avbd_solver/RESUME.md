# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

The official VBD and AVBD PDFs, project pages, source repositories, and paper
videos were audited into an exact 176-row coverage contract. VBD now has 88
canonical rows (methods, limitations, every figure/table/video segment, Gaia
dynamics/cloth configurations, TinyVBD, and every distinct project-page media
surface); AVBD has 88 (methods, every figure/table/video segment, every 2D/3D
source scene, both deployed demos, and every distinct project-page media
surface). No row is complete under the fail-closed rule. A checker and mutation
tests reject source-pin drift, deleted rows, invalid evidence paths, media
timecode drift, incomplete backend/predicate proof, and premature overall
completion.

## Current Branch

`feature/vbd-avbd-paper-parity-contract`, based on the then-current
`origin/main` at `83110ef54ab`. The branch is local and has not been pushed.

## Immediate Next Step

Finish the contract packet with:

```bash
pixi run check-plan104-paper-parity
pixi run python -m pytest -q tests/test_check_plan104_paper_parity.py
pixi run check-avbd-packets
pixi run lint
```

Then implement the first missing core paper mechanism,
`avbd.method.quasi_newton_hessian`, with focused equation/kernel mutation tests
before attempting dependent scenes. Do not promote any dependent row without
source-matched CPU and CUDA solver-identity, physical, visual, and comparable
performance evidence.

## Context That Would Be Lost

- **Keep claims narrow.** Do not claim a source-demo/paper CPU win, GPU parity,
  a broad breakable-wall/fracture corpus, a same-hardware paper-number match, or
  an all-coefficient friction win unless the tracked artifacts directly prove it.
- The durable completion rule is
  [`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md);
  the older corpus matrix's `Complete` labels describe narrow historical assets,
  not canonical parity rows.
- Two profile-build allocator gate failures
  (`DantzigSolver.ScratchUsesProvidedAllocatorForDantzigWorkBuffers`,
  `World.BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator`) are pre-existing
  LCP / rigid-IPC issues outside AVBD scope; they reproduce only under
  `DART_BUILD_PROFILE=ON` and hosted CI is green. Do not attribute them to AVBD.
- PLAN-104's progress log + git history hold the full per-slice record; this
  folder no longer duplicates it.

## How to Resume

```bash
git checkout main && git pull --ff-only
git status --short --branch
# Build if needed (AVBD tests import the compiled extension):
pixi run build
```

Then use the contract row blockers and PLAN-104 `AVBD Current Next Gaps` to
choose the next slice. Verify with the relevant focused `test_avbd_*` /
`test_world` / `test_variational_integration` targets plus the contract checker
and `pixi run lint`.
