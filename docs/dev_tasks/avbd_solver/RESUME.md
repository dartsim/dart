# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

The exact 176-row VBD/AVBD paper-parity contract remains fail-closed, with no
complete rows. The latest local packet extends the CPU articulated variational
bridge from passive finite world-link rows to same-multibody movable-link
spherical/revolute/prismatic pairs and bounded finite revolute/prismatic
`Velocity` motors. Their masked constrained coordinates use persistent
per-axis AVBD stiffness state while the free coordinate uses a motor-only
projection row with the existing hard-motor target/Jacobian/effort bounds.

The durable packet is
[`../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-motors-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-motors-packet.json).
It records passive and motor same-multibody C++ oracles, all three warmed
allocation policies, a Python command/reversal oracle, assessed docked
start/middle/end visual evidence, candidate-only 2/8/32-motor scale data, and
an exact-parent mutation run. The motor behavior test fails both driven
coordinates on exact parent `761263bbd41` and passes on the candidate. The
linked method rows remain partial.

The Release build, complete variational binary (183/183), aggregate C++ unit
tier (168/168), full Python tier (1651 passed, 20 skipped), six packet-writer
tests, PLAN-104 parity contract (176 rows), all 54 AVBD packets, and PLAN-122
allocation matrix (18 rows, 14 closed) pass. The simulation label passes 77/79
active entries: its split global-heap and raw-malloc binaries reproduce the
same four unrelated 320/80-byte and 320/1376-byte signatures recorded on the
exact parent, while every new finite articulated allocation gate passes. Two
monolithic entries remain intentionally disabled.

## Current Branch

`feature/vbd-avbd-paper-parity-contract`, based on the then-current
`origin/main` at `83110ef54ab`. The fail-closed contract is committed locally
at `710cbfc1152`, the Section 3.5 packet at `a78f688a178`, and the Section 4
packet at `0b0154573b8`; the articulated finite-row packet is committed at
`761263bbd41`, and the articulated finite-motor packet is the current
uncommitted closeout. The branch has not been pushed.

## Immediate Next Step

After aggregate gates and the local closeout commit, continue articulated
extraction with finite-row load accounting and fracture lifecycle. Preserve
the packet boundary: the current CPU finite rows do not close unified rows,
CUDA, the source/paper corpus, or achieved-accuracy performance.

## Context That Would Be Lost

- **Keep claims narrow.** Do not claim a source-demo/paper CPU win, GPU parity,
  a broad breakable-wall/fracture corpus, a same-hardware paper-number match, or
  an all-coefficient friction win unless the tracked artifacts directly prove it.
- The articulated finite-motor benchmark is candidate-only. Exact parent skips
  the new rows, so a timing ratio would compare different work and is not a
  speedup claim.
- Finite public one-DOF `Velocity` rows now use a separate bounded motor-only
  row beside their compliant masks. Do not attach break-force source state to
  that row until finite compliant and motor loads have a complete accounting
  contract; doing so now would be partial and misleading.
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
pixi run check-plan122-allocation-matrix
```

Stay in this exact worktree and branch. Verify the current packet with focused
AVBD/world allocation/Python writer and scene tests, the parity and allocation
contract checkers, aggregate build/unit gates, and `pixi run lint`; do not push
or mutate GitHub without explicit approval.
