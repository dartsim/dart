# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

The exact 176-row VBD/AVBD paper-parity contract remains fail-closed, with no
complete rows. The latest local packet closes the narrow CPU break-load gap for
same-multibody finite articulated point joints. Accepted finite-row forces and
bounded motor projection loads now share public force/torque units, aggregate
into one per-joint L2 threshold, clear finite row state on fracture, skip while
broken, and re-engage after reset.

The durable packet is
[`../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-fracture-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-fracture-packet.json).
It records finite-only, motor-only at two timesteps, combined-load,
break/reset/re-arm, and save/load C++ oracles; all three warmed allocation
policies; a Python lifecycle oracle; assessed docked start/middle/end visual
evidence; candidate-only 2/8/32-motor scale data; and an exact-parent mutation
run. Both load/lifecycle tests fail in five required cases on exact parent
`9ebd9b895b1` and pass on the candidate. The linked method rows remain partial.

The focused load/lifecycle/serialization C++ oracles, all existing AVBD
breakage tests, all three warmed allocation policies, the Python scene oracle,
six packet-writer tests, assessed 149-frame docking capture, and pinned
candidate-only benchmark pass. The Release build, complete variational binary
(186/186), aggregate C++ unit tier (168/168), full Python tier (1658 passed,
20 skipped), PLAN-104 parity contract (176 rows), AVBD packets (55), and
PLAN-122 allocation matrix (18 rows, 14 closed) also pass. The simulation label
passes 77/79 active entries; only the split global-heap and raw-malloc binaries
retain the four unrelated exact-parent-reproduced allocation signatures, while
all three new finite articulated gates pass inside those splits.

## Current Branch

`feature/vbd-avbd-paper-parity-contract`, based on the then-current
`origin/main` at `83110ef54ab`. The fail-closed contract is committed locally
at `710cbfc1152`, the Section 3.5 packet at `a78f688a178`, and the Section 4
packet at `0b0154573b8`; the articulated finite-row packet is committed at
`761263bbd41`, and the articulated finite-motor packet at `9ebd9b895b1`; the
articulated finite-fracture packet follows as the current local unpushed
closeout. The branch has not been pushed.

## Immediate Next Step

Broaden the paper wall/fracture and joint corpus, unified rigid/soft rows, or
another highest-priority missing paper mechanism. Preserve the packet
boundary: this narrow CPU load contract does not close those rows, CUDA, the
source/paper corpus, or achieved-accuracy performance.

## Context That Would Be Lost

- **Keep claims narrow.** Do not claim a source-demo/paper CPU win, GPU parity,
  a broad breakable-wall/fracture corpus, a same-hardware paper-number match, or
  an all-coefficient friction win unless the tracked artifacts directly prove it.
- The articulated finite-motor benchmark is candidate-only. Exact parent skips
  the new rows, so a timing ratio would compare different work and is not a
  speedup claim.
- Finite public one-DOF `Velocity` rows use a bounded motor-only projection row
  beside their compliant masks. Its `dt^2` position-level load must remain
  associated with the owning compliant constraint and converted to physical
  units before comparison; do not regress to an isolated motor threshold or
  compare raw projection lambda to public break force.
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
