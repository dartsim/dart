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
  `feature/vbd-avbd-paper-parity-contract`, owning draft PR #3432. Do not copy a
  mutable head SHA into this tracker: verify `git rev-parse HEAD`, fetched
  `origin/main`, live PR head/draft state, CI, and reviews immediately before
  any external action. Keep the PR draft until the foundation gates and review
  loop are clean; never infer merge approval from an earlier session.
- **Current packet:** C++ and dartpy callers can explicitly select public
  Sequential Impulse, VBD, and AVBD rigid-body families with one positive
  contact/joint solve budget. Sequential Impulse now owns hard fixed,
  spherical, revolute, and prismatic pair rows, velocity motors,
  impulse-derived breakage, and non-velocity post-stabilization in the same
  projected Gauss-Seidel sweeps as contacts. Shared rigid-pair geometry and
  input extraction now have solver-neutral ownership, while SI and AVBD keep
  separate row/dual state. The in-step private-AVBD fallback under public SI
  invalidates AVBD contact and hard-joint continuation state while preserving
  independently active AVBD distance-spring continuation (regression
  `World.SequentialImpulseFallbackPreservesAvbdDistanceSpringWarmStart`).
  A solver-family crossing through `setRigidBodySolver` likewise preserves
  that spring continuation, and the springs ramp and warm-start on the same
  paper-profile schedule under both families, so the step after a crossing is
  the uninterrupted step (regression
  `World.RigidAvbdDistanceSpringScheduleIsContinuousAcrossFamilyCrossing`).
  Finite-stiffness pair rows fail closed to VBD/AVBD. VBD uses
  fixed finite-penalty rows without dual accumulation or progressive
  stiffness; AVBD retains
  augmented-Lagrangian dual/stiffness state. VBD and AVBD still reject
  unsupported boxed-LCP or multibody envelopes instead of silently falling
  back. The matched publication-shaped Figure 13 demos share 252 staggered
  bricks, 712 attachments, three 40 kg impacts at 24 m/s, 1/60 s, 20 sweeps,
  and fingerprint `8ca3fbfa00c3dce9`, but carry distinct paper-shaped outcome
  oracles.
  The contracts contain 88 VBD and 88 AVBD requirements; all 176 remain
  incomplete until their recorded correctness, solver-identity, CPU/CUDA,
  visual, and comparable-performance predicates pass.
- **Current packet chain (schema 6, sealed on the final source bytes):**
  [`../../plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json)
  links the matched AVBD/VBD packet to an independently assessed Sequential
  Impulse frame-14 fracture and frame-120 collapse oracle. Exact joint
  identities and retained-row residuals are recorded at every checkpoint.
  Nine captures (three per method, including one 600-frame long-horizon
  still/video each) and their semantic reviews are sealed to capture-source
  digest
  `115a185b7ae0f2d122f227503091da5236990716160ad9263595a6f880671a43`
  with the embedded Git HEAD of the sealed source commit. One quiet-host
  five-repeat Release run through `scripts/run_figure13_benchmark.py`
  recorded median CPU costs of 16.050815 ms AVBD, 13.973706 ms VBD,
  and 14.499464 ms Sequential Impulse, with CPU CVs of 0.99%,
  0.65%, and 0.28%, respectively. Packet hashes are
  validated transitively by `check-avbd-packets`; this tracker intentionally
  does not duplicate those mutable values. The same-host ratios are
  descriptive costs, not speedup or achieved-accuracy equivalence claims.
  The AVBD row's outcome changed under the immutable paper profile (see its
  packet section below): the wall stays standing with three localized
  joint-break clusters, so its semantic review records visual agreement with
  Figure 13(d) as not proven.
  Figure 13 and video row 12 remain partial: exact source constants, XPBD, a
  source-matched four-method edit, CUDA, and achieved-accuracy reference
  performance are open.
- **Recent slices merged to `main`** (see the PLAN-104 progress log and the PRs
  for detail; per-slice history lives in git, not in this file):
  - #2991 — source-row coverage + contact-precheck (`f6fecbc5bd5`).
  - #3004 — 2D/3D Spring/Spring Ratio contact-filtering, inertia-orientation
    cleanup, refreshed packets, contact-skip regressions (`356384967f8`).
  - #3018 — box edge/vertex rigid-contact feature-ID coverage (`6bf7b2e8336`).
  - #3022 — bounded regression coverage: rigid-contact tangent-basis contract,
    articulated break→reset→break re-arm lifecycle, row-inventory replaced-key
    cold-start (`65ba05113c6`).
- **Current local gates:** on the sealed source bytes the Release C++ suite
  passes 310/310 `ctest` entries (including `test_boxed_lcp_contact` 133/133,
  `test_avbd_rigid_block` 141/141, `test_world` 504/504,
  `test_world_resolved_configuration` 37/37), the evidence-script unit tests
  pass, and `tests/test_check_avbd_packets.py` passes once the three Figure 13
  packets are regenerated. The full default `pixi run test-all` and the CUDA
  `pixi run -e cuda test-all` gates are recorded in the RESUME stop point
  when they have run on the final bytes; a green CUDA environment gate still
  would not close the missing solver-specific VBD/AVBD GPU-parity predicates.

## Foundation PR Exit Criteria

Draft PR #3432 is ready for maintainer review only when every item below is
true on one final source snapshot. These criteria close the shared foundation;
they do not claim that any of the 176 paper-parity rows is complete.

1. The public Sequential Impulse, fixed-penalty VBD, and augmented VBD/AVBD
   selections execute their advertised formulation without an implicit family
   fallback. Runtime diagnostics and schema-6 packets bind the actual selected
   family, formulation, contact method, iteration policy, projection policy,
   and scene fingerprint.
2. The zero-trust correctness audit has no unresolved P1/P2 finding in the
   foundation envelope. In particular, rigid and deformable contact candidate
   construction is bounded and fail-closed, per-contact friction continuation
   cannot alias another contact, fixed-penalty VBD cannot retain AVBD dual or
   anchor state, attachment/fracture state has one explicit solver-row metric
   and lifecycle contract with its force/torque dimensional limitation stated,
   and replay rejects every construction/configuration/layout drift that can
   change continuation meaning.
3. Every current packet passes the current schema-6 validator. Historical
   schemas remain readable only through the exact filename/version allowlist;
   no historical packet may close a current claim. Writer-owned source lists,
   source digests, solver fingerprints, artifact hashes, parent links, and
   semantic reviews all validate transitively.
4. The matched Figure 13 AVBD, VBD, and Sequential Impulse rows are regenerated
   from the final binaries. Each has the method-specific diagnostic checkpoint
   oracles, a 600-frame (10 simulated seconds) long-horizon capture, a still,
   a video, engine and pixel-integrity verdicts, and an independent semantic
   review bound to the exact bytes. The evidence is publication-shaped partial
   evidence only: outcomes may differ, and no speedup or source-equivalence
   claim is permitted without a matched accuracy denominator.
5. One uncontaminated same-host five-repeat benchmark run records all three
   methods from the same scene/configuration, with the quiet-host and in-run
   watchdog gates passing. The PR description reports medians, dispersion,
   hardware/build identity, exact commands, outcome differences, and honest
   limitations; it embeds or links the long-horizon videos and still fallbacks.
6. `pixi run lint`, the full default `pixi run test-all`, the full visible-GPU
   `pixi run -e cuda test-all`, focused replay/serialization/allocation/contact
   regressions, both packet/parity validators, and the text-first plus visual
   simulation gates pass on the final bytes. A CUDA build/environment pass is
   not mislabeled as solver-specific VBD/AVBD CUDA parity.
7. Immediately before the final push, the latest fetched `origin/main` is
   merged (never rebased) into the published branch and all final-byte gates
   affected by that merge are rerun. Required hosted checks are green, the PR
   remains draft, its DART 7.0 milestone and description are current, and the
   complete `@codex review` loop has no actionable finding. Merging still
   requires separate explicit maintainer approval.

After #3432 lands, full paper parity remains exactly two owning changes: one
VBD PR closing all 88 VBD rows and one AVBD PR closing all 88 AVBD rows. Each
must include its complete implementation, CPU/CUDA correctness tests, all key
paper/project/video/source demos, achieved-accuracy performance comparisons,
benchmarks, and long-horizon visual evidence; neither program may be split into
implementation-only or evidence-only follow-ups.

## Goal

Implement AVBD as the hard-constraint continuation of DART's VBD solver family:
all paper/reference algorithms and features, CPU and GPU parity, complete
paper/site/video/demo reproduction in DART tests/benchmarks/`py-demos`, and
performance that beats the reference demo repositories and published paper
numbers.

## Non-Goals For Current Phases

- Keep the public AVBD surface to the DART-owned method-family selector and
  validated iteration policy; do not expose row storage, solver registries,
  CUDA types, or ECS details.
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

The foundation head is the takeover commit chain recorded in
[`RESUME.md`](RESUME.md) (rigid packet, evidence pipeline, re-derived AVBD
oracle, evidence-runner fixes, then the evidence commit). If the foundation PR
is still open, manage only that PR through current-head CI and review; do not
extend its branch into the remaining parity program. After
maintainer approval and landing, complete all 88 VBD predicates in one VBD PR
from updated `main`, including an honest public XPBD comparator, the VBD
CPU/CUDA and corpus gaps, achieved-accuracy performance, and PR-hosted images,
GIFs, and videos for every paper/site/video/source-demo row. After that lands,
complete all 88 AVBD predicates in a second AVBD PR from updated `main`,
including AVBD-specific CPU/CUDA, unified-row, corpus, four-method,
performance, and PR-hosted visual evidence. These are the only two follow-up
PRs: do not split implementation, performance, or replication media into
separate PRs. Keep every dependent figure/demo/performance row partial until
its own source-matched CPU and CUDA evidence closes it.

Deferred maintenance items remain valid but do not outrank the missing paper
mechanism:

- hoist the duplicated `makeCollisionPairKey` into a shared `detail` header;
- upgrade the Spring / Spring Ratio packets from legacy schema version 1 to
  the current solver-identity contract;
- consolidate the duplicated pair-constraint mask/basis/orientation math into
  one owner (drift is currently pinned by a kernel `static_assert` and the
  `RigidPairConstraintNeutralHelpers` equivalence test) and route the SI
  finite-stiffness admission rule through a solver-neutral accessor;
- revisit two recorded SI conventions: kinematic joint endpoints are treated
  as zero-velocity (shared with the contact path, so a kinematic-driven
  jointed body lags), and joint post-stabilization runs after contact
  position correction, so it can re-introduce shallow penetration that the
  wall oracles budget; and
- consolidate SI per-step joint-view walks, skip SI container reserves for
  non-SI families, and record post-stabilization work in the step-iteration
  diagnostic.

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

- **Value:** give finite articulated rows the same solver-row break metric as
  hard and bounded motor rows, so configured thresholds are stable across
  timesteps and one joint fractures from its complete active-row metric.
- **Scope:** evaluate accepted finite load as stiffness times residual, convert
  position-projection lambda to force/torque with `1 / dt^2`, aggregate all
  finite and motor row coordinates by L2 norm, clear finite row state on
  fracture, and preserve broken/reset policy through simulation-mode
  save/load. Linear-force and angular-torque coordinates are not normalized by
  a characteristic length, so this is not a physical-unit wrench norm.
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

## Current-Source Packet: Public AVBD Figure 13 Wall

- **Value:** make supported free-rigid AVBD contact and pair constraints an
  explicit public `World` family, then use that exact family for the first
  publication-shaped breakable-wall outcome.
- **Public contract:** `RigidBodySolver::Avbd` /
  `sx.RigidBodySolver.AVBD`, a positive `RigidConstraintOptions::iterations` /
  `sx.RigidConstraintOptions(iterations=...)` policy, binary and replay
  persistence, resolved-family reporting, and fail-closed rejection of
  unsupported or incompatible configuration.
- **Scene/oracle:** 252 staggered bricks, 712 breakable fixed attachments,
  three 40 kg balls launched at 24 m/s, 1/60 s, and 20 projection sweeps.
  Under the immutable paper profile (alpha 0.95 on contact rows as well) the
  balls lodge in the wall instead of rebounding and the anchored wall
  transmits the impulse to the ground, so at frame 120 the deterministic
  oracle records 36 broken and 676 unbroken attachments, no brick displaced
  beyond the 0.1 m damage threshold, and 100% outside-wall and total
  retention. The same broken-identity digest is recorded at frames 60, 120,
  and 600. Of those identities, `[5, 5, 5]` have initial anchors within the
  three selected 1.15 m impact regions and 21 lie outside; this packet
  therefore claims three localized joint-break clusters and a standing wall,
  not the broken-open wall of Figure 13(d), and its semantic review records
  visual agreement with that panel as not proven. The 676 surviving rows stay
  below 1.63 mm linear and 0.00119 rad angular residual at frame 120 (the
  regularized rows still relax the impact deformation). Frame 60 remains
  explicitly pre-evaluation even though every threshold check already
  passes there; the registered AVBD claim checkpoint is frame 120.
- **Runtime evidence:** the public AVBD contact-plus-breakable-row fixture
  passes world-base, global-`new`, and raw-malloc first-post-bake gates. The
  frame-60, frame-120, and 600-frame software captures bind the exact front
  camera and pass engine ViewReports, pixel integrity, and image-capable
  semantic review against the pinned paper page; the long-horizon video
  decodes to 600 frames at 60 fps. The scene and benchmark share fingerprint
  `8ca3fbfa00c3dce9`. Capture manifests bind the sealed capture-source
  digest and embedded Git HEAD, the loaded runtime images, the screenshot,
  and the scene-metrics log; the benchmark JSON binds the same source tree,
  its benchmark translation unit, the evidence build configuration, the
  quiet-host gate, and the in-run watchdog.
- **Performance boundary:** the five-repeat Release benchmark on the sealed
  bytes records a 16.050815 ms median CPU cost per step with
  0.99% CPU CV. This is an absolute DART timing only; the paper
  and source publish no directly comparable timing for this scene.
- **Non-goals:** exact replay of unpublished source constants, the XPBD
  comparison row, source-matched video edit, broad fracture corpus, unified
  rows, and CUDA. The matched VBD and Sequential Impulse rows are tracked
  separately below.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-paper-breakable-wall-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-breakable-wall-packet.json).
Figure 13 and official-video row 12 remain partial.

## Current-Source Packet: Matched Public VBD / AVBD Figure 13 Rows

- **Value:** expose an honest public fixed-penalty VBD family over the same
  DART-owned rigid 6-DOF block infrastructure, then compare it with public AVBD
  using one exact reconstructed scene rather than relabeling sequential
  impulse.
- **Public contract:** `RigidBodySolver::Vbd` /
  `sx.RigidBodySolver.VBD`, the same positive rigid-constraint iteration
  policy, binary/replay persistence, resolved-family reporting, and
  fail-closed boxed-LCP and multibody rejection. VBD uses finite penalty
  stiffness, zero dual state, and no progressive stiffness update.
- **Matched outcome evidence:** both demos and benchmark share fingerprint
  `8ca3fbfa00c3dce9`, 252 bricks, 712 attachments, three balls, 1/60 s, and 20
  sweeps. VBD records no fracture, 0.133 m peak wall-normal and 0.063 m RMS
  displacement with 123 bricks beyond 0.05 m at frame 18, then 100% retained
  attachments at frames 120 and 600. All 712 retained rows stay below
  16.90 mm and 0.01342 rad at frame 18, then 11.10 mm and 0.001310 rad at
  frame 120. All three VBD captures pass their engine ViewReports,
  pixel-integrity checks, and paper-grounded semantic review.
- **Performance boundary:** the five-repeat medians on the sealed bytes are
  13.973706 ms VBD and 16.050815 ms AVBD, with 0.65% and
  0.99% CPU CV. The VBD/AVBD median CPU-cost ratio is
  0.8706x and is descriptive only: the outcomes intentionally
  differ, and no source achieved-accuracy or same-hardware denominator
  exists.
- **Remaining boundary:** exact unpublished constants, honest XPBD, a
  source-matched four-method edit, CUDA, broad
  fracture/unified rows, and reference-performance parity remain open.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-paper-vbd-comparison-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-vbd-comparison-packet.json).
Figure 13 and official-video row 12 remain partial.

## Current-Source Packet: Matched SI / VBD / AVBD Figure 13 Rows

- **Value:** replace the default contact-only label with solver-owned public
  hard pair rows and use that actual Sequential Impulse family for the third
  matched Figure 13 row.
- **Public contract:** hard fixed/spherical/revolute/prismatic masks and
  bounded one-DOF velocity motors interleave with contact rows across the
  configured projected Gauss-Seidel sweeps. Break force is derived from
  accumulated impulse over `dt`; a non-velocity post-stabilization pass reduces
  pose drift. Finite-stiffness public pair rows reject SI and direct callers to
  VBD/AVBD.
- **Matched outcome evidence:** frame 14 records exactly 5 broken/707
  unbroken joints. Their impact-region counts are `[2, 1, 2]` with none
  outside, `[13, 14, 13]` bricks are displaced in the three impact bands, and
  100% of brick placement remains. Frame 120 has the same broken-identity
  digest and no additional fracture, but the 484 retained rows outside all
  impact regions reach 0.06346 m maximum / 0.02076 m RMS linear residual and
  0.7270 rad maximum / 0.2164 rad RMS angular residual. Only 27.27% outside
  the impact bands and 19.05% overall remain placed, with 2.878 m maximum
  wall-normal displacement, and the 600-frame capture keeps the collapsed
  wall. All three SI captures match the paper's initial fracture followed by
  retained-row failure and collapse.
- **Performance boundary:** the five-repeat median CPU costs on the sealed
  bytes are 14.499464 ms SI, 13.973706 ms VBD, and 16.050815 ms
  AVBD, with 0.28%, 0.65%, and 0.99% CPU
  CV. SI/AVBD is 0.9033x and SI/VBD is 1.0376x.
  These are
  descriptive costs for intentionally different outcomes, not paper speedups
  or achieved-accuracy comparisons. Frame 120 is the only shared quantitative
  checkpoint across the three rows; the earlier frame-14/18/60 checkpoints
  are per-family diagnostics.
- **Remaining boundary:** XPBD, exact unpublished constants, a source-matched
  four-method edit, CUDA, broad fracture/unified rows, and reference-
  performance parity remain open.

The durable evidence is
[`../../plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json).
Figure 13 and official-video row 12 remain partial.

## History

Per-slice history and durable evidence live in the PLAN-104 progress log, the
paper gap audit, the corpus matrix, git history, and the merged PRs above. Do
not re-accrete a session-by-session log in this file; keep it to current state.
