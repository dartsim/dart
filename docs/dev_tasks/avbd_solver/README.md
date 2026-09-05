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
- **Branch state:** the shared foundation merged to `main` in PR #3432
  (`aafc4b66072`, 2026-09-03). No feature branch is open; the VBD-completion
  PR starts from updated `main`. Verify `git rev-parse HEAD`, fetched
  `origin/main`, and live PR/CI state immediately before any external action;
  never infer merge approval from an earlier session.
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
- **Checked-in evidence footprint:** the 71 JSON files under `docs/` (4.0
  MB) are each named by a fail-closed validator, a writer test, or the plan
  page that cites them; none is an orphan. The three Figure 13 packets are
  0.47 to 0.55 MB each, a fifth of which is the per-frame PNG digest list
  that makes them self-verifying against the capture roots. Slimming them
  would mean binding those digests behind one root digest, a schema-7
  contract decision rather than a cleanup, so the packets stay as sealed.
- **Current packet chain (schema 6, sealed on the final source bytes):**
  [`../../plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json`](../../plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json)
  links the matched AVBD/VBD packet to an independently assessed Sequential
  Impulse frame-14 fracture and frame-120 collapse oracle. Exact joint
  identities and retained-row residuals are recorded at every checkpoint.
  Nine captures (three per method, including one 600-frame long-horizon
  still/video each) and their semantic reviews are sealed to capture-source
  digest
  `5a1c911e685ccb45033e072a04355bff8bc14c52b7e16dee431d6f0d958f554e`
  and record the sealed source commit's Git HEAD (the validators recompute
  the digest; the recorded head is format-checked, not looked up in git).
  One quiet-host
  five-repeat Release run through `scripts/run_figure13_benchmark.py`
  recorded median CPU costs of 13.252252 ms AVBD, 11.268830 ms VBD,
  and 14.906923 ms Sequential Impulse, with CPU CVs of 0.52%,
  0.31%, and 0.43%, respectively. Packet file hashes are
  validated transitively by `check-avbd-packets`; this tracker quotes only the
  capture-source seal digest above and does not duplicate the mutable packet
  hashes. The same-host ratios are
  descriptive costs, not speedup or achieved-accuracy equivalence claims.
  The AVBD row's outcome changed under the immutable paper profile (see its
  packet section below): the wall stays standing with three localized
  joint-break clusters, so its semantic review records visual agreement with
  Figure 13(d) as not proven. The zero-trust audit ladder had made the
  public AVBD and VBD Figure 13 steps about 1.75x and 1.5x slower than the
  pre-audit head; the memoized rigid block kernel (D4: the SO(3)
  left-Jacobian inverses of a joint's orientation error built once per body
  visit, the cached world points of a quasi-Newton point-pair row shared by
  its value, direction, and geometric term, bitwise the per-row values)
  brings the sealed medians back by 12 % and 14 % (13.25 / 11.27 ms against
  15.07 / 13.04 ms on the previous seal); the remaining gap to the pre-audit
  head is the exact SO(3) Jacobian and the fail-closed manifold identity,
  both kept on purpose. Sequential Impulse is 7 % slower than on the
  previous seal (14.91 against 13.93 ms); the only change on its path is the
  box-box manifold, which now hands it the touching corners within the skin
  that the reference-plane clip used to drop, an attribution by elimination
  rather than by a per-row profile.
  Figure 13 and video row 12 remain partial: exact source constants, XPBD, a
  source-matched four-method edit, CUDA, and achieved-accuracy reference
  performance are open.
- **Named AVBD parameter profiles (D5):** the public AVBD family selects
  one named, immutable profile (`RigidAvbdParameterProfile`): the paper's
  Table 2 by default, or the pinned 2D/3D reference-demo defaults. The
  selection is recorded in the resolved configuration and bound into replay
  and binary checkpoints (format 36). The two source profiles reproduce the
  pinned sources' solver rules, each verified against a headless build of
  the reference source itself: PENALTY_MIN 1 / PENALTY_MAX for every public
  row (the `kStart`/`kMax` of the resolved note; a hard joint between equal
  light bodies stalls the block sweep at a large start stiffness, in the
  source exactly as here), the adaptive initial guess of Algorithm 1 line 4
  (the sources' `accelWeight`, fed by a per-body projected-velocity history
  that is part of the warm-start replay state), the sources' contact
  `COLLISION_MARGIN` rest depth, the joint `torqueArm` scale of the angular
  rows, the sources' Coulomb-cone rules (latest normal dual in 2D, normal
  trial force in 3D), feature-only manifold continuation, and, for the 2D
  source, post-stabilization (main sweeps with alpha 1 on every row, full
  dual warm start, one extra primal-only sweep with alpha 0 applied to the
  transforms after the velocities are taken). Free-falling hard-jointed
  pairs, the heavy rope, resting and penetrated boxes, and the rod now match
  the reference sources to three or four decimals under those profiles.
- **D6 decided (default profile):** the paper leaves the row start
  stiffness `k_start` free and no fixed value serves every scale: the
  sources' PENALTY_MIN 1 lets a 1000 kg box sink 0.14 m before its penalty
  ramps, while DART's former fixed 1e5 stalled the block sweep of light
  hard-jointed bodies (a pair hovered at 1 % of free fall; the reference
  sources at PENALTY_MIN 1e5 reach only 92 %). The public default is now
  `RigidAvbdParameterProfile::MassScaledReference`: the `avbd-demo3d`
  source's rules and constants with every contact and joint row starting at
  its reduced mass over dt^2 (`kStartScale=1` in the resolved note; a fixed
  body counts as infinite mass), which makes the first-step penetration and
  the regularized approach to the 1 cm margin identical for 1 kg, 1000 kg,
  and 10 t boxes (2.8 mm, creeping to 7.8 mm in two seconds) and keeps a
  light jointed pair on the free-fall trajectory to 0.2 mm over a second.
  `Paper2025Table2` stays available with its Table 2 constants, DART's 1e5
  start, and its step-start sweep origin (Table 2's beta of 10 cannot ramp
  hard joints from a small start in SI units, and the adaptive guess at a
  1e5 start pushes landing structures through their supports); the Figure 13
  wall and the empty baseline select it explicitly, so their sealed evidence
  is unchanged (36 breaks, [5, 5, 5], 21 outside, same digest).
  `RigidAvbdJointedPairFallsLikeAFreeBody` and
  `RigidAvbdMassScaledStartRestsHeavyAndLightBoxesAlike` pin the behaviours.
  Two boundaries of the default are recorded: the adaptive guess carries no
  gravity until two projections exist (the sources' `accelWeight`), so a
  hard-jointed pair released from rest keeps a one-time residual of 4e-4 m/s
  and 6e-4 rad/s from its first two sweeps under the mass-scaled start
  (1e-5 under the sources' PENALTY_MIN 1; nothing accrues afterwards), and
  the profile-derived row construction (mass-scaled start, adaptive guess,
  `torqueArm`) belongs to the AVBD family only: fixed-penalty VBD and the
  per-body compatibility path under Sequential Impulse keep their configured
  stiffness and their step-start sweep origin. The Eq. 18 decay contract
  tests select `Paper2025Table2` explicitly.
- **Finding E closed, finding F closed for sustained friction (source rows):** all 31 source-demo
  scenes run their reference profile, the sources' ten solver iterations
  (their row metadata recorded the number while the worlds ran DART's
  default eight), and a 1.0 depth for the 2D ports (a thinner port let the
  separating-axis test pick the out-of-plane axis for the source scenes'
  spawn overlaps, sinking the 2D fracture pillars and the stack-ratio
  boxes). The 2D ports lock their bodies to the plane before every step
  (`examples/demos/_avbd_demo2d_plane.py`, declared
  `replay_live_step_is_stateless`): the reference solver has no
  out-of-plane degrees of freedom, and without the lock the 2D fracture and
  static-friction piles tilted out of the plane after a second. The twelve
  source rows whose thresholds had been calibrated on the pre-audit private
  contact configuration are re-derived from the headless reference runs
  (`tests/integration/test_demos_cycle.py` cites the source numbers per
  row); none is parked. The 3D ramp placement matches the source (its boxes
  start 0.13 m inside the ramp there too).
  Finding F was the port's weak sustained friction: the 2D static-friction
  pile crept 2.3 m down its 30 degree ramp in three seconds where the
  reference holds within 3 cm. Row-level traces against the reference found
  four causes, each fixed at its owner: the port applied the 3D source's
  per-step slip threshold to the 2D profile, whose source keeps a friction
  anchor while its dual is strictly inside the cone and the anchored points'
  step-start tangential offset is below STICK_THRESH
  (`frictionStickOffsetThreshold`, `stickOffset=0.01` in the resolved note);
  the box-box face clip applied the reference plane as a fifth clip plane,
  so a box rocking by microradians produced a crossing vertex that slid
  along the face edge every step and churned the contact features; the
  contact feature classifier mapped a corner to whichever of its three faces
  rounding made nearest; and the AVBD identity rule cold-started a whole
  manifold whenever its point count changed. The clipper now keeps each
  incident vertex within a skin of the reference plane with its own depth
  (the sources' per-point separation), the classifier keeps corners and
  edges as corners and edges, and feature-only identity continues each
  persisting key on its own. One, two, and three slabs now rest where the
  reference rests them (`(-0.0079, 0.8609)`, `(-0.0114, 1.4356)`,
  `(-0.0295, 2.0019)` against `(-0.0079, 0.8609)`, `(-0.0111, 1.4358)`,
  `(-0.0362, 1.9981)`), the eleven-slab pile holds (its bottom slab drifts
  1.5 cm in three seconds, the reference's 3.2 cm), and the source rows
  still pass with their reference bounds. What stays open of finding F is
  sliding friction: the 2D dynamic-friction box with mu 5 on the mu 0.5
  ground now decelerates from 10 m/s to 5.3 m/s in a second (8.2 m/s
  before these fixes) where the reference reaches 1.59 m/s; the reference
  cold-starts a sliding manifold every step because its feature matching
  fails there and re-ramps the rows within the step, while the port keeps
  the warm-started rows, and that row bounds the port's values and cites the
  source's.
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
  passes every `ctest` entry (including `test_boxed_lcp_contact` 133/133,
  `test_avbd_rigid_block` 142/142, `test_world` 510/510,
  `test_world_resolved_configuration` 38/38, `test_serialization` 83/83,
  `test_box_box` 34/34, `test_vbd_contact` 35/35), the evidence-script unit
  tests pass (863 validator tests), and `tests/test_check_avbd_packets.py`
  passes on the regenerated Figure 13 packets. On `29900ea732c` (the
  evidence commit plus the regenerated box-on-ground default-step golden,
  whose dropped box now settles 3 um lower under the per-vertex box
  manifold) the full default `DART_DISABLE_COMPILER_CACHE=ON pixi run
test-all` passed every phase (229 + 81 `ctest` entries; its Python phase:
  2014 passed, 20 skipped, no expected failures), and `pixi run -e cuda
test-all` passed every phase (213 + 80 + 8 `ctest` entries; its Python
  selection does not include the demo-cycle rows). A green CUDA environment
  gate still would not close the missing solver-specific VBD/AVBD
  GPU-parity predicates.

## Foundation Landed

PR #3432 closed the shared foundation on 2026-09-03: honest public Sequential
Impulse, fixed-penalty VBD, and augmented VBD/AVBD selections without implicit
family fallback, schema-6 packets bound to the selected family and scene
fingerprint, the zero-trust correctness audit with no open P1/P2 finding in
the foundation envelope, matched Figure 13 rows regenerated from the final
binaries with long-horizon captures and semantic review, and one uncontaminated
same-host five-repeat benchmark. None of that claims any of the 176 paper-parity
rows.

Full paper parity remains exactly two owning changes: one
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

The shared foundation is on `main` (PR #3432, `aafc4b66072`); no foundation
branch or PR remains. Complete all 88 VBD predicates in one VBD PR from
updated `main`, including an honest public XPBD comparator, the VBD
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
  diagnostic; and
- revisit the `Paper2025Table2` profile's `beta`/`k_start` units if its
  Table 2 constants are ever to run with the adaptive initial guess (the
  default profile already does; see D6).

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
improves 1.02%; the 2D/3D Spring mechanisms cost 0.43%/1.90% more. These are
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
  digest and record its Git HEAD, the loaded runtime images, the screenshot,
  and the scene-metrics log; the benchmark JSON binds the same source tree,
  its benchmark translation unit, the evidence build configuration, the
  quiet-host gate, and the in-run watchdog.
- **Performance boundary:** the five-repeat Release benchmark on the sealed
  bytes records a 13.252252 ms median CPU cost per step with
  0.52% CPU CV. This is an absolute DART timing only; the paper
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
  11.268830 ms VBD and 13.252252 ms AVBD, with 0.31% and
  0.52% CPU CV. The VBD/AVBD median CPU-cost ratio is
  0.8503x and is descriptive only: the outcomes intentionally
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
  bytes are 14.906923 ms SI, 11.268830 ms VBD, and 13.252252 ms
  AVBD, with 0.43%, 0.31%, and 0.52% CPU
  CV. SI/AVBD is 1.1249x and SI/VBD is 1.3228x.
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
