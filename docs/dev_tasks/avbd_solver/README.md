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
- Fail-closed VBD/AVBD parity contract:
  [`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md),
  backed by the machine-checked VBD and AVBD JSON inventories in the same
  directory.

## Current Status

- **Active, incomplete paper implementation.** No parity is claimed. The
  headline gates remain **open**: a source-demo/paper CPU win, GPU parity, and a
  same-hardware paper-number match.
- **Branch state:** active local branch
  `feature/vbd-avbd-paper-parity-contract`, based on `main` at `83110ef54ab`;
  the fail-closed contract is committed locally at `710cbfc1152`, and nothing
  has been pushed. The Section 3.5 packet is committed locally at
  `a78f688a178`, and the Section 4 packet at `0b0154573b8`; the articulated
  finite-row packet at `761263bbd41`; the articulated finite-motor packet at
  `9ebd9b895b1`; the articulated finite-fracture packet is the current local
  unpushed closeout.
- **Current packet:** same-multibody finite-stiffness articulated point joints
  now account for accepted finite-row force and bounded motor projection load
  in public force/torque units. All rows owned by a joint contribute to one L2
  threshold; fracture clears finite row state, later steps skip the broken
  joint, reset re-engages it, and weak re-arm can break it again.
  The contracts contain 88 VBD and 88 AVBD requirements; all 176 remain
  incomplete until their recorded correctness, solver-identity, CPU/CUDA,
  visual, and comparable-performance predicates pass.
- **Latest verified local packet:** focused finite-only, motor-only
  two-timestep, combined-load, break/reset/re-arm, and save/load behavior;
  warmed allocation; Python scene; exact-parent mutation; docked assessed
  visual; and candidate-only scale evidence advance
  `avbd.method.joints_and_attachments`,
  `avbd.method.finite_stiffness_ramping`, `avbd.method.motors`, and
  `avbd.method.fracture` without completing any row. The paper wall, broad
  joint/fracture corpus, unified rows, CUDA, and source-matched
  achieved-accuracy performance remain open.
- **Recent slices merged to `main`** (see the PLAN-104 progress log and the PRs
  for detail; per-slice history lives in git, not in this file):
  - #2991 — source-row coverage + contact-precheck (`f6fecbc5bd5`).
  - #3004 — 2D/3D Spring/Spring Ratio contact-filtering, inertia-orientation
    cleanup, refreshed packets, contact-skip regressions (`356384967f8`).
  - #3018 — box edge/vertex rigid-contact feature-ID coverage (`6bf7b2e8336`).
  - #3022 — bounded regression coverage: rigid-contact tangent-basis contract,
    articulated break→reset→break re-arm lifecycle, row-inventory replaced-key
    cold-start (`65ba05113c6`).
- **Current local gates:** the focused finite-only, bounded-motor,
  combined-load, break/reset/re-arm, and save/load C++ oracles; all 55 AVBD
  breakage tests; all three articulated finite-row post-bake allocation
  policies; six packet-writer tests; and the focused Python lifecycle oracle
  pass. The two mutation-sensitive C++ tests fail in five required cases on
  exact parent `9ebd9b895b1`. The assessed 149-frame docking-build capture and
  pinned 2/8/32-motor candidate-only benchmark are recorded in the packet.
  The Release build, complete variational binary (186/186), aggregate C++ unit
  tier (168/168), full Python tier (1658 passed, 20 skipped), PLAN-104 parity
  contract (176 rows), AVBD packets (55), and PLAN-122 allocation matrix
  (18 rows, 14 closed) pass. The simulation label passes 77/79 active entries;
  only the split global-heap and raw-malloc binaries fail on the same four
  unrelated signatures previously reproduced on exact parent (320/80 global
  bytes and 320/1376 raw bytes), while all three new finite articulated gates
  pass inside those splits. Two monolithic entries remain intentionally
  disabled.

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

Continue PLAN-104 with the paper wall/broader fracture and joint corpus,
unified rigid/soft rows, or another highest-priority missing paper mechanism.
Keep every dependent figure/demo/performance row partial until source-matched
CPU and CUDA evidence closes it.

Two smaller deferred maintenance items remain valid but do not outrank the
missing paper mechanism: hoist the duplicated `makeCollisionPairKey` into a
shared `detail` header, and upgrade the Spring / Spring Ratio packets from
legacy schema version 1 to version 2 with a `resolved_solver_identity`.

## Verified Local Packet: Section 3.5 Quasi-Newton Hessian

- **Value:** replace DART's current rank-one-only or generic PSD-clamped AVBD
  row blocks with the paper's non-negative diagonal approximation of the
  force-scaled geometric stiffness so the inertia-anchored local block remains
  positive-definite.
- **Scope:** add one allocation-free shared column-norm kernel; use it for
  deformable finite-stiffness distance springs, rigid distance springs, rigid
  point attachments, and nonlinear rigid point-pair rows used by linear
  joints and motors. Encode the paper/reference implementation's intentional
  Taylor-linearized contact exception in row data so normal and friction
  contact rows do not silently acquire a different Hessian model.
- **Non-goals for this packet:** angular log-map curvature, CUDA kernels,
  second-order contact curvature, new paper scenes, broad performance claims,
  and closing `avbd.method.quasi_newton_hessian` as complete. These remain in
  the overall task rather than disappearing from scope.
- **Assumptions and decisions:** the geometric matrix is diagonally lumped by
  the Euclidean norm of each column after force scaling, exactly as AVBD
  Section 3.5 specifies. The pinned 2D solver applies that rule generically and
  the 3D joint source applies it to nonlinear point rows; the pinned 3D spring
  source omits its geometric term, so DART's distance-spring implementation
  follows the paper without claiming 3D source equivalence. The rank-one term
  always uses the current penalty stiffness. Contact remains linearized because
  both the paper and reference source explicitly discard its second-order term.
- **Acceptance evidence:** analytic expected-matrix tests for compressed and
  oblique deformable springs, off-center rigid attachments and point pairs,
  rigid springs with rotational curvature, and explicit contact linearization;
  at least one mutation-sensitive test that fails if the geometric diagonal is
  removed; matched before/after fixed-joint and spring benchmark runs; focused
  post-bake allocation coverage; a text-first solver/scene oracle; and an
  assessed headless capture if the behavior-level scene changes visibly.
- **Gates:** `pixi run lint`, focused AVBD unit tests, the relevant DART 7 world
  allocation filters, `pixi run build`, parity-contract validation, matched
  benchmark comparison, and two clean review passes.
- **Dependencies:** PLAN-104 paper contracts and gap audit, PLAN-122 allocation
  contract, the pinned AVBD paper/source revisions, and the existing rigid and
  deformable AVBD row inventories.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-quasi-newton-evidence.json`](../../plans/104-vertex-block-descent-solver/avbd-quasi-newton-evidence.json).
The fresh focused result is 115/115 rigid AVBD and 9/9 deformable
finite-stiffness tests, plus all three post-bake allocation policies. The
60-second source Spring oracle remains finite and settles within 0.00018 m of
the expected static length. The current-build fixed-joint mechanism benchmark
improves 1.02%; the 2D/3D Spring mechanisms cost 2.32%/1.90% more. These are
descriptive same-host costs, not paper/reference-performance claims. The
120-frame software render passed pixel integrity and manual semantic review.

## Verified Local Packet: Section 4 Parallel Dual/Stiffness Update

- **Value:** match Algorithm 1's additional post-primal pass without
  serializing otherwise independent AVBD rows.
- **Scope:** factor one deterministic, allocation-stable CPU pass over the
  promoted deformable and rigid row inventories; preserve row order and exactly
  match serial dual, bounds, fracture, and stiffness state.
- **Non-goals:** CUDA closure, new row families, changes to public APIs,
  nondeterministic reductions, and a paper-speedup claim before matched
  achieved-accuracy evidence.
- **Acceptance evidence:** serial/parallel state equivalence for every promoted
  row kind, thread-count determinism, warmed-step world-base/global/raw
  allocation gates, failure propagation, and matched throughput measurements.
- **Claim boundary:** this can advance
  `avbd.method.parallel_dual_stiffness_pass` only to partial until all paper row
  families and CUDA share the same contract.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-parallel-dual-update-evidence.json`](../../plans/104-vertex-block-descent-solver/avbd-parallel-dual-update-evidence.json).
Focused compute/deformable/rigid coverage passes 228/228, and the three new
production-World activation/allocation tests pass. The exact-parent interleaved
benchmark keeps 8,192 rows inline and records current 2-/4-worker speedups of
1.66x/2.20x at 16,384 rows, 2.34x/3.77x at 32,768 rows, and 1.78x/3.90x at
65,536 rows. These are descriptive mechanism-throughput results, not
paper/reference-performance claims. No new render is required for this
scheduling-only packet because bitwise serial/parallel row and production
World state equivalence is the stronger behavior oracle.

## Verified Local Packet: Articulated Finite Masked Rows

- **Value:** make public passive finite-stiffness spherical, revolute, and
  prismatic world-link point joints affect the variational articulated solve
  instead of being silently skipped.
- **Scope:** reuse persistent per-axis AVBD ramp state for masked linear and
  angular rows on CPU; preserve spherical rotation, revolute hinge rotation,
  and prismatic axis translation as free coordinates.
- **Non-goals:** finite one-DOF velocity-motor coupling, same-multibody finite
  endpoint pairs, finite-row fracture accounting, unified soft/rigid rows,
  CUDA, and a paper/reference speedup claim.
- **Correctness evidence:** the public S/R/P behavior oracle passes on the
  candidate and fails five constrained-coordinate assertions on exact parent
  `0b0154573b8`; the allocation-free fixed-size 6x6 articulated inverse-mass
  path matches a dense nine-DOF solve.
- **Runtime evidence:** warmed world-base, global-`new`, and raw-malloc gates
  pass; the Python text oracle and assessed start/middle/end render show the
  constrained bodies remain aligned while their free coordinates move.
- **Performance boundary:** the pinned candidate-only benchmark records 71.7
  us for 3 joints, 333.5 us for 12, and 2.404 ms for 48. The parent skipped
  these rows, so the packet makes no before/after speedup claim.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-joints-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-joints-packet.json).
Both linked canonical method rows remain partial.

## Verified Local Packet: Articulated Finite Movable-Pair Motors

- **Value:** make passive finite masked rows work between two movable links of
  one multibody and make finite revolute/prismatic `Velocity` joints drive
  their free coordinate instead of being silently passive.
- **Scope:** retain compliant constrained coordinates and add one bounded
  motor-only projection row for each finite revolute/prismatic free coordinate,
  using the hard-motor target, Jacobian, and effort-bound semantics.
- **Non-goals:** finite-row break-force/load accounting, unified soft/rigid
  rows, CUDA, and a paper/reference speedup claim. The motor-only row
  intentionally does not attach a `sourceJoint` until load accounting is
  complete.
- **Correctness evidence:** non-cardinal, off-origin same-multibody passive and
  motor oracles cover spherical/revolute/prismatic masks plus strong/tiny
  revolute/prismatic effort limits. The motor oracle fails both driven
  coordinates on exact parent `761263bbd41`.
- **Runtime evidence:** warmed world-base, global-`new`, and raw-malloc gates
  pass; the Python text oracle validates forward/reversed commands and bounded
  residuals; the docking-build start/middle/end render shows both movable-pair
  mechanisms and their live metrics.
- **Performance boundary:** the pinned candidate-only benchmark records 104.2
  us for 2 motors, 579.8 us for 8, and 5.803 ms for 32. The parent skipped
  these rows, so the packet makes no before/after speedup claim.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-motors-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-motors-packet.json).
The linked method rows remain partial.

## Verified Local Packet: Articulated Finite Load And Fracture

- **Value:** give finite articulated rows the same physical break-force
  contract as hard and bounded motor rows, so public thresholds are stable
  across timesteps and one joint fractures from its complete row load.
- **Scope:** evaluate accepted finite load as stiffness times residual, convert
  position-projection lambda to force/torque with `1 / dt^2`, aggregate all
  finite and motor rows by L2 norm, clear finite row state on fracture, and
  preserve broken/reset policy through simulation-mode save/load.
- **Correctness evidence:** finite-only, motor-only at 5 ms and 10 ms, and
  combined-load threshold oracles; break/skip/reset/re-arm and binary
  round-trip lifecycle coverage; all existing AVBD breakage regressions; and
  all three warmed finite articulated allocation policies.
- **Mutation evidence:** both load/lifecycle tests fail in the five required
  cases on exact parent `9ebd9b895b1` and pass on the candidate.
- **Runtime evidence:** the
  `avbd_articulated_compliant_breakable_motor` text oracle and assessed docked
  capture show weak break, strong reset/intact, and weak re-arm/break phases.
- **Performance boundary:** the pinned candidate-only benchmark records 103.2
  us for 2 breakable motors, 585.0 us for 8, and 6.633 ms for 32. The parent
  omitted this load accounting, so the packet makes no speedup claim.
- **Non-goals:** the paper wall, broad fracture/joint corpus, unified
  rigid/soft rows, CUDA, and source/paper achieved-accuracy performance.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-fracture-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-articulated-compliant-fracture-packet.json).
All linked canonical method rows remain partial.

## History

Per-slice history and durable evidence live in the PLAN-104 progress log, the
paper gap audit, the corpus matrix, git history, and the merged PRs above. Do
not re-accrete a session-by-session log in this file; keep it to current state.
