# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

Recent AVBD slices merged to `main` (#2991, #3004, #3018, #3022): source-row
coverage and contact-precheck, 2D/3D Spring/Spring Ratio contact-filtering with
inertia-orientation cleanup and refreshed packets, box edge/vertex rigid-contact
feature IDs, and a bounded mutation-verified regression pass (tangent-basis
contract, articulated break→reset→break re-arm, row-inventory replaced-key
cold-start). No parity was claimed; the CPU-win, GPU, and paper-number gates
remain open.

## Current Branch

`main` — clean, synced with `origin/main`. There is **no** active local AVBD
follow-up branch; the next slice starts fresh from current `main`.

## Immediate Next Step

Pick one bounded, mutation-verified slice from PLAN-104's `AVBD Current Next
Gaps` / the corpus matrix, or one of the two ready deferred items in
[`README.md`](README.md) (hoist the duplicated `makeCollisionPairKey` into a
shared `detail` header; upgrade the Spring/Spring Ratio packets to
`schema_version` 2 with a `resolved_solver_identity`). Verify with the focused
targets and `pixi run lint`; keep claims narrow.

## Context That Would Be Lost

- **Keep claims narrow.** Do not claim a source-demo/paper CPU win, GPU parity,
  a broad breakable-wall/fracture corpus, a same-hardware paper-number match, or
  an all-coefficient friction win unless the tracked artifacts directly prove it.
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

Then choose the next slice from PLAN-104 `AVBD Current Next Gaps` and verify with
the relevant focused `test_avbd_*` / `test_world` / `test_variational_integration`
targets plus `pixi run lint`.
