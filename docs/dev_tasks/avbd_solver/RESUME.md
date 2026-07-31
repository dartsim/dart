# Resume: AVBD Solver

Working handoff for PLAN-104's Augmented Vertex Block Descent implementation.
The durable roadmap, evidence, next gaps, and slice history live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
this file is only the current session pointer. See
[`README.md`](README.md) for status, goal, decisions, and next steps.

## Last Session Summary

The exact 176-row VBD/AVBD paper-parity contract remains fail-closed, with no
complete rows. The latest local slice makes supported free-rigid AVBD contact
and pair constraints explicitly selectable through the C++ and dartpy
`RigidBodySolver` family. Its positive contact/joint projection budget is
public, serialized, replay-safe, runtime-mutable, and reported in resolved
configuration when the split rigid stage is active. Mixed semi-implicit
multibody worlds use the unified constraint stage, accept only the default
rigid constraint options, and report that split-stage budget as not
applicable; unsupported or incompatible combinations fail closed.

The durable packet is
[`../../plans/104-vertex-block-descent-solver/avbd-paper-breakable-wall-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-breakable-wall-packet.json).
It binds that public solver identity to a publication-shaped Figure 13 scene:
252 staggered bricks, 712 breakable attachments, three high-momentum balls,
1/60 s, and 20 projection sweeps. The deterministic frame-120 oracle records
359 broken/353 unbroken attachments, `[4, 10, 6]` local displaced-brick
counts, 91.16% outside-wall retention, and 82.54% total retention. Frame 60 is
explicitly pre-evaluation because its `[3, 9, 4]` counts leave one band below
the final threshold. Both captures record the exact front camera and pass
engine ViewReports, pixel integrity, and image-capable semantic review against
the pinned paper page. The runtime scene and benchmark share fingerprint
`2a746821cc10faee`; the five-repeat Release benchmark records 7.739 ms median
CPU time per step.

The public solver/schedule/serialization/Python tests, both scene oracles, all
three public-AVBD first-post-bake allocation policies, all 16 fail-closed
packet-writer tests, the full 273-test migrated packet/schema set, all 56
working-tree AVBD packets, deterministic reruns, captures, and benchmark pass.
The final broad default gates pass: `pixi run build`, all 168 unit targets, all
445 `test_world` cases, all 15 resolved-configuration cases, all 20 contact-
parity cases, and 1,677 Python tests with 20 expected skips. The full CUDA
`test-all` passes all seven phases, including the GUI-disabled Python catalog,
documentation, all eight CUDA runtime tests, and CUDA benchmark smoke. Packet
provenance was regenerated after formatting and all 16 writer, 273 aggregate,
56 working-tree packet, and 176-row parity gates pass with zero source-hash
mismatches. The PLAN-122 allocation matrix passes with 18 rows, 14 closed.

The final dual review found one additional contract mismatch: non-default
rigid iteration options were accepted and reported as applied in mixed
semi-implicit multibody worlds even though their unified constraint stage did
not consume that budget. The runtime now rejects the combination at
construction finalization, option/method mutation, load, and step entry; a
mixed active-contact regression proves the default budget is reported as not
applicable. The focused post-fix gates pass: 15/15 resolved-configuration,
445/445 World, 61/61 serialization, 17/17 schedule, and 168/168 aggregate unit
tests. Both post-fix review lanes approve the result. Exact final reruns pass
the uncached default build, the 1,677-test default Python suite, and all seven
CUDA `test-all` phases; the eight CUDA runtime tests include the 208.75-second
Jacobi batch case.

The broad gates found two candidate defects that are now covered. The boxed-
LCP runtime had allocated a full differentiable snapshot only to recover
contact forces; it now uses the allocation-free apply path and copies the
existing force vector, restoring the zero-allocation World gate. The first
CUDA Python pass also found that the Figure 13 scene constructed
`OrbitCamera` when GUI bindings were disabled. Headless builds now expose
`view_assessment_available: false` and `view_report: null`, while GUI builds
still require the positive engine ViewReport used by the evidence writer.

## Current Branch

`feature/vbd-avbd-paper-parity-contract`, based on the then-current
`origin/main` at `83110ef54ab`. The fail-closed contract is committed locally
at `710cbfc1152`, the Section 3.5 packet at `a78f688a178`, and the Section 4
packet at `0b0154573b8`; the articulated finite-row packet is committed at
`761263bbd41`, the articulated finite-motor packet at `9ebd9b895b1`, and the
articulated finite-fracture packet at `131981788fa`. The public AVBD/Figure 13
slice is the current local unpushed closeout. The branch has not been pushed.

## Immediate Next Step

Extend the wall into the source-matched Sequential Impulse, XPBD, and VBD
comparison rows and CUDA row, or proceed to unified rigid/soft and the
remaining fracture/joint corpus. Preserve the packet boundary: the current row
is a publication-shaped AVBD CPU reconstruction, not exact source, full video,
four-method, CUDA, or comparable performance closure.

## Context That Would Be Lost

- **Keep claims narrow.** Do not claim a source-demo/paper CPU win, GPU parity,
  a broad breakable-wall/fracture corpus, a same-hardware paper-number match, or
  an all-coefficient friction win unless the tracked artifacts directly prove it.
- Figure 13 does not publish the exact wall dimensions, mass/inertia choices,
  ball speed/targets, or break threshold. The DART constants are explicitly
  reconstructed and must not be described as an exact source replay.
- The benchmark is absolute DART timing. There is no paper/reference timing
  denominator for this scene, so do not derive a speedup or parity ratio.
- The outcome image's optional contrast diagnostic records 1,019 bright
  pixels against a 1,180-pixel heuristic. The required non-blank gate and
  engine ViewReport pass, and the separate semantic review adjudicates
  fracture morphology. Do not recast the optional diagnostic as a failed
  render or suppress it from the packet.
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
pixi run build
```

Stay in this exact worktree and branch. Verify the current packet with focused
AVBD/world allocation/Python writer and scene tests, the parity and allocation
contract checkers, aggregate build/unit gates, and `pixi run lint`; do not push
or mutate GitHub without explicit approval.
