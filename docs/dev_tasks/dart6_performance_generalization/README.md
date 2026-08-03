# DART 6 performance generalization (issue #3056, round 2)

> **Status: ACTIVE — planning approved, packets open.** Single source of truth
> for the second round of DART 6.20 performance work on
> [issue #3056](https://github.com/dartsim/dart/issues/3056) (reopened
> 2026-06-30). Round 1 (#3123..#3203, closed out by #3199) solved the
> mostly-resting 3k-shapes regime; this round targets the **general active
> contact-rich regime**, the **gz-default ODE backend**, the **penetration-creep
> root cause that prevents piles from sleeping**, and **large-island solver
> scalability**, while coordinating with the built-in `dart` collision
> detector.
> Baseline branch point: `origin/release-6.20` @ `70b92010311` (includes
> #3209 benchmark, #3229 dart/simd, #3230 dashboard, #3226 deactivation gate).

## North-star completion gate

This task is not complete merely because the current packet queue becomes
gated. It completes only when issue #3056 is closure-ready for the DART 6.20
line with evidence that the fix is general, not overfit to one reproducer.
That means the final closeout must include:

- focused regression tests for any solver, contact, collision, sleeping, or
  benchmark behavior changed by the round;
- #3307-style benchmark reports covering the issue-specific 3k-shapes/gz
  default ODE path, the active contact-container fixture, settled-scene
  regression guards, and untouched-detector hash/contact/resting checks;
- representative headless and GUI example evidence from `contact_benchmark`
  (`--gui-capture`, sleep-state colors, and final-scene dumps where useful)
  so reviewers can inspect whether the optimized scene is physically
  plausible, finite, and not merely faster; and
- a recorded decision for every behavior-changing gate that remains relevant
  to #3056 (especially D3, D7, and D8), with an explicit maintainer-approved
  deferral if a gate is not fixed on this branch.

Do not retire this folder, park PLAN-621, or post a final issue-closure claim
while any required evidence above is missing.

## Why this round exists

Round 1 made settled scenes fast (3k_shapes `dart` detector RTF ~81, hash
`0x131b6af79a44ff90`, 3003/3003 resting; ODE RTF 0.05 → 18.1). What it did
not claim, and what the #3209 benchmark + PR body make measurable and
explicit, is the **active** regime and why scenes stay active:

- Active 120-body container: `dart` detector RTF ≈ 0.036, ODE ≈ 0.021; 16
  threads change almost nothing (see
  [01-baseline-evidence.md](01-baseline-evidence.md)).
- Profile smoke (active 60-body, `dart`): the Dantzig LCP solve scope ≈ 66%
  of step time, collision ≈ 8%. Caveat: this maps to the solver's own
  profile scope; WP-PG.01/WP-PG.10 must record the `Construct LCP` vs
  solve-proper split before WS-A effort is committed.
- **Penetration creep blocks sleeping** (#3209 root-cause finding 2):
  baseline contact error correction is capped at `DART_MAX_ERV = 1e-3` m/s
  (`ContactConstraint.cpp:49`); piles compact faster than the cap corrects,
  so max penetration grows ~5–10 mm/s without bound — and the island-rest
  veto requires every contact ≤ 1e-5 m penetration
  (`kSleepContactPenetrationTolerance`, `ConstraintSolver.cpp:2050`,
  `World.cpp:1229`), so **creeping piles can never freeze**. Detector- and
  solver-independent; deterministic reproducer recorded in
  01-baseline-evidence.md (scene S6). WP-PG.15 addresses this for dense
  mobile-mobile islands while preserving legacy behavior for single-mobile
  static-support islands.
- gz-sim uses the **ODE collision detector by default** (issue #3056,
  @azeey); the DART-side ODE wrapper carries quadratic contact-history
  bookkeeping, and its trimesh cylinder fallback can flood contact caps
  (#3209 root-cause finding 3).

## Workstreams

| ID | Lane | Owner doc | Packets | Thrust |
| --- | --- | --- | --- | --- |
| WS-A | Constraint/LCP pipeline | [02-constraint-lcp-lane.md](02-constraint-lcp-lane.md) | PG.10–PG.15 | The active-regime wall: instrumentation, scan removal, direct assembly, islanding evidence, matrix-free option, creep/rest-veto root cause |
| WS-B | ODE backend | [03-ode-backend-lane.md](03-ode-backend-lane.md) | PG.20–PG.23 | De-quadraticize contact history; manifold reduction on the gz default backend |
| WS-C | Dynamics batching + allocation | [04-dynamics-batching-lane.md](04-dynamics-batching-lane.md) | PG.30–PG.33 | Single-free-body fast paths, scratch retention, frame-arena discipline, SoA cohorts |
| WS-D | SIMD enablement | [05-simd-enablement-lane.md](05-simd-enablement-lane.md) | PG.40–PG.42 | Make merged `dart/simd` (#3229) earn its keep at proven seams; FP/ISA contracts first |
| WS-E | Infra + evidence | [06-infra-evidence-lane.md](06-infra-evidence-lane.md) | PG.01–PG.04 | Durable baselines, profiling doc, benchmark matrix extensions, executor tooling |
| WS-F | DART collision backend | [DART 6 collision backends](../../design/dart6_collision_backends.md) | lifecycle | Delivered for DART 6.20 by #3381; any default or dependency change is later-release work |
| WS-G | MuJoCo cross-engine comparison | [08-mujoco-comparison-lane.md](08-mujoco-comparison-lane.md) | harness + gap packets | Maintainer-directed generalization bar: DART must outperform MuJoCo across arms, humanoids, many-object, sleeping, and highly dynamic workloads (full pipelines; no sleep-shortcut wins on dynamic classes) |

Status across lanes lives in
[07-orchestration-dashboard.md](07-orchestration-dashboard.md).
Session handoff lives in [RESUME.md](RESUME.md). Prior experiments
(including measured-and-rejected approaches on the `origin/perf/dart6-*`
branches) are inventoried in 01-baseline-evidence.md — **read before
claiming any packet that overlaps them**.

## Success criteria (provisional until WP-PG.01 re-baselines; maintainer ratifies)

1. **Primary fixture** (active container, `dart` detector, 1 thread,
   120 objects): cumulative default-on packets reach **RTF ≥ 3× the round-2
   baseline** (0.036 → ≥ 0.108) with all guard hashes stable.
2. **Pile-sleep outcome** (the gz-visible #3056 closer, needs D7/WP-PG.15):
   the S6 creep reproducer (71-object container, 20 s) ends with bounded
   `max_penetration` and all bodies resting under default settings.
3. **No regressions**: settled 3k `dart` stays ≥ round-2 baseline RTF
   within noise (±5%); settled 3k ODE row does not regress; every guard
   scene keeps bit-identical hashes for untouched detectors.
4. **General evidence trigger**: issue-specific wins are insufficient. The
   final report must include broad tests, benchmark matrices, and GUI/headless
   example evidence proving the optimized path is not overfit to one fixture.
5. **Completion trigger**: criteria 1–4 met (or maintainer-approved
   exceptions recorded), D3/D4/D5/D7/D8 resolved with their packets landed
   or explicitly deferred, issue #3056 ready for closure, then the closeout
   promotions below and folder deletion in the completing PR.

## Compatibility envelope (hard rules for every packet)

gz-physics/gz-sim backward compatibility is the critical constraint. The
gz dartsim plugin subclasses `OdeCollisionDetector` and
`ContactSurfaceHandler`, swaps detectors by string
(`"fcl"/"bullet"/"ode"/"dart"`), reads
`World::getLastCollisionResult()`, dynamic_casts to
`BoxedLcpConstraintSolver`, and uses `BodyNode::moveTo` detach flows
(#3227). Therefore:

1. **Default-on changes must be deterministic**: bit-identical final-state
   hash, contact count, pair count, and resting count per collision
   detector on the guard scenes, following the #3188–#3194 pattern
   (version-keyed caches, fast paths auto-bypassed for non-default
   handlers/solvers).
2. **Behavior-changing packets** (PG.14, PG.15, PG.23) are a separate PR
   class: gated on their maintainer decision, carrying a tolerance
   rationale, old/new guard rows, and explicit re-baseline sign-off. They
   never share a PR with behavior-preserving work.
3. **Parallelism stays opt-in** (`World::setNumSimulationThreads`, default
   1; gz never enables it).
4. **cpp-only where possible**; additive private members allowed only until
   6.20.0 is first packaged (SOVERSION freezes at MAJOR.MINOR), and vtables
   of gz-subclassed classes (`CollisionDetector`, `OdeCollisionDetector`,
   `ContactSurfaceHandler`, `BoxedLcpConstraintSolver`) are **frozen now**.
5. **SoA/aligned scratch lives behind pimpl or function-local state**, never
   in public class layouts; alignment-touching packets must run
   `pixi run test-eigen-overalignment`.
6. **No `-march` flags on exported targets** (DART_ENABLE_SIMD precedent);
   `dart/simd` backend selection is compile-time, so packaged binaries get
   baseline ISA unless WS-D delivers runtime dispatch (Decision D2).

## Mandatory per-packet gates

- `pixi run lint` (auto-fix) **plus** `pixi run check-lint` (verify — this
  is what CI enforces; checking only the sub-linter you touched misses
  failures).
- Capped C++ build gate for Codex/release hosts:
  `pixi run cmake --build build/default/cpp/Release --target ALL --parallel 8`.
  Do not use `pixi run test-all` as the capped substitute: that task still
  runs an unbounded `cmake --build ... -j --target ALL` (see `pixi.toml`).
- `pixi run -e gazebo test-gz` for anything touching collision, constraint,
  solver, `World::step`, or public headers.
- Determinism guard: `contact_benchmark` final-state hash + contact/pair/
  resting counts vs the recorded baseline for **every** detector (`dart`,
  FCL, Bullet, ODE) — untouched backends must be bit-identical.
- Benchmark evidence in the #3307 format: every performance PR must carry a
  PR-body performance report, not just a small timing table. The report must
  include the benchmark evidence head, parent/current-base comparison commits,
  exact commands, benchmark settings, an apples-to-apples equivalence scope,
  a strict regression gate or explicit maintainer-approved exception, compact
  before/after graphs when the table is non-trivial, and a full raw matrix
  with samples, mean/median timing, speedup/change, contacts/resting/cap or
  final-hash guard columns as applicable. Label non-equivalent detector rows
  as diagnostic instead of counting them as winners/regressions. ODE rows are
  only valid with `--max-contacts-per-pair 4` (#3209 finding 3). RTF-only or
  best-row-only acceptance is banned.
- Packet metadata: compiler, CPU/governor, pixi env, exact commands, and
  which optional detectors were built (per the collision-backend design).
- `pixi run test-eigen-overalignment` when allocation/alignment changes.

## Sequencing

```
Wave 0 (prerequisite):        WP-PG.01 (baseline packet)
Wave 1 (parallelizable now):  WP-PG.02, WP-PG.03, WP-PG.40 (resolves D1/D2),
                              WP-PG.10, WP-PG.11, WP-PG.20, WP-PG.22,
                              WP-PG.30, WP-PG.31
Wave 2 (dependency-driven):   WP-PG.12 (after PG.10, ideally PG.30),
                              WP-PG.21 (after PG.20),
                              WP-PG.41 (after PG.40 + PG.10 seams),
                              WP-PG.32
Evidence-gated:               WP-PG.13 (only if PG.10's island census shows
                              groups coarser than contact connectivity)
Decision-gated:               WP-PG.04 (D4), WP-PG.23 (D8),
                              WP-PG.33 (PG.30; D1/D2 for the SIMD variant)
WS-F shipped as the built-in `dart` detector through #3381. It has no active
performance-generalization packet; later default or dependency changes follow
the collision-backend lifecycle design.
```

One packet = one branch (`wp-pg-<nn>-<slug>`) = one PR
(`WP-PG.<nn>: ...`) = one verification story. Never stack PRs on parent
PR branches. Claim packets by marking the dashboard row and RESUME.md.

## Non-goals (explicit, evidence-backed)

- Re-adding body sleeping (default-on and island-correct since #3086; #3226
  scaled the final-quiet gate with configured thresholds).
- SIMD rewrites of Dantzig inner loops or fixed-size `ContactConstraint`
  Eigen math (already vectorized; the cost is structural).
- Taskflow/ComputeExecutor adoption, ECS Model/State split, SoA state
  storage, GPU offload (DART 7-only; this workload is
  compute_backend_research.md "Pattern B": threads + SIMD + allocation
  discipline + islanding granularity).
- Split-impulse tuning (off by default in 6.20).
- Porting DART 7 collision-world plumbing (EnTT).
- Re-attempting the measured-and-rejected round-1 experiments (see the
  prior-art inventory in 01-baseline-evidence.md) without new evidence.

## Open decisions (maintainer input needed before dependent packets)

- **D1 — FP-determinism contract for SIMD/FMA** (blocks WP-PG.41+): default
  posture is bit-identical scalar/SIMD results on state-affecting paths
  (no reassociation; consistent FMA usage); hash re-baselining only with
  maintainer approval and recorded old/new hashes. The gz gate's
  ChangedWorldPoses patch is exact-equality, so pose-path drift fails CI.
- **D2 — ISA delivery** (blocks WP-PG.41+): accept packaging-baseline ISA
  gains only, or build a small runtime-dispatch layer (per-backend TUs +
  CPU feature check). Naive per-file `-mavx2` on unconditional code would
  SIGILL on older CPUs.
- **D3 — Matrix-free large-island solver**: resolved by merged #3361
  (`WP-PG.14: add opt-in matrix-free contact LCP`). DART 6.20 keeps dense
  Dantzig as the default and exposes matrix-free contact PGS only as an
  explicit default-off `BoxedLcpConstraintSolver` option plus
  `contact_benchmark`/dartpy controls. The #3353/D7 default remediation
  satisfies the primary-fixture default-on target, so D3 is not promoted to a
  default behavior on 6.20. Future work may tune or broaden the option only
  with explicit option-off hash guards and option-on finite/convergence/
  re-baseline evidence because matrix-free PGS has different solver semantics
  from Dantzig.
- **D4 — Executor tooling**: keep `/dart-resume` + RESUME.md as the 6.20
  entry point, or backport a 6.20-adapted `dart-execute-packet` command
  (WP-PG.04).
- **D5 — ODE lane depth**: proposal — land the bounded set WP-PG.20/21/22
  (bookkeeping/algorithmic, behavior-preserving) and stop there pending
  WS-F phase 5 (facade decision). WP-PG.23 is governed by D8, not D5.
- **D6 — Deactivation default divergence**: 6.20 defaults deactivation ON,
  main defaults OFF. Flagged for awareness; no action proposed this round.
- **D7 — Penetration-creep remediation policy**: resolved by merged #3353. The
  behavior-changing default policy promotes the evidenced contact ERV/tolerance
  values through dense-island adaptive policy, keeps ordinary single-body
  support contacts on the legacy effective ERV/tolerance path, adds
  dense-contact-island sleep candidacy for sub-wake jitter, and preserves the
  strict legacy policy for explicit
  `setAutomaticSleepingContactPenetrationTolerance(1e-5)` calls. D3 remains the
  separate default-off large-island solve-side option.
- **D8 — Contact-manifold reduction on current detectors** (blocks
  WP-PG.23): round 1 explicitly deferred default-on manifold
  reduction/selection (recorded FCL evidence: 2.6× RTF from per-pair
  capping). The DART-owned detector now has its own contact-manifold policy;
  the remaining decision is whether to pursue a behavior-changing packet on
  the ODE/FCL wrappers. Deciding *not* to do it now must be recorded as a
  decision, not an omission.
- **D9 — DECIDED 2026-08-01 (maintainer-delegated, evidence-based): SHIP
  the detector stream-quality bundle** — full 4-contact face manifolds,
  stable two-point cylinder side-line contacts (with the tilted-support
  effective-radius correction), retained shallow crossed-cylinder
  contacts, and spin-invariant aligned cylinder-box handling, as one
  cohesive WP-PG.50 packet on `wp-pg-wsg-rebaseline-20260731`. Rationale:
  every piece removes performance-from-lost-or-unstable-contacts, which
  the `docs/design/dart6_collision_backends.md` correctness clause
  forbids; the complete bundle measures **0.946x vs the audited
  pre-consolidation stack on the primary fixture (direct quiet-host
  interleave — slightly faster)** with fuller manifolds, keeps criterion
  1 at ≈3.7x (≥ 3x bar; the only cross-session link is the standing
  audit record, whose trajectory the audit arm reproduces bit-exactly),
  flips ARM-PUSHER to a cross-engine win, settles S4/S5 4x faster, and
  leaves FCL/Bullet/ODE bit-identical. Evidence:
  01-baseline-evidence.md 2026-07-31/08-01 sections.
- **D10 — DECIDED 2026-08-01, superseded for the better on the same day:
  criterion 2 is MET on its original terms.** The root cause of the pile
  never resting was a chain of detector stream defects, the last being
  the aligned cap-patch path demanding exact rotational identity so
  spun-but-upright cylinders fell to degenerate convex rim points and
  crept without bound (measured on the intermediate build: seed 101 rising from ~0.016 m to
  0.137 m across a 60k run). With the full bundle, S6 ends **71/71
  resting with max penetration 0** on 60k runs of both the canonical
  seed and the worst creeper, and 4/5 seeds fully deactivate within the
  original 20k window (the audit-era stack managed 3/5); canonical 3056
  freezes between 36k–38k steps (sleep latency; penetration stays in a
  bounded ~1–3.6 mm band with no growth trend until the freeze). An intermediate re-anchoring of criterion 2 (recorded in
  this entry's history via git) proved unnecessary once the final defect
  fell. Measured-and-rejected alternative: raising the dense-island rest
  tolerance (premature island freezing under load explodes penetration
  to 0.256 m — a freeze-under-load hazard to remember). Non-blocking
  follow-ups recorded: deactivation-latch latency for large
  single-island piles near stillness (a dartpy-authored 0.1-scale
  box-only pile converges to 5e-5–1.3e-3 m/s without latching in 40k
  steps while contact_benchmark-authored piles do latch), rolling
  friction as a physical feature for roller-heavy scenes, and
  parallel-line single-point contacts (cyl-cyl/capsule pairs) as the
  remaining 1-point supports. Evidence: 01-baseline-evidence.md
  "2026-08-01 spin-canonicalization"; seed matrices and 60k trends in
  `~/dart-wsg-evidence-20260731/aug01/`.

## Closeout plan (promotion targets, decided up front)

Dev-task folders are deleted at completion in the completing PR. Durable
outputs and their owners:

- Benchmark baselines + tolerance envelopes → performance dashboard
  artifacts (#3230 workflow) and
  `docs/readthedocs/community/performance_dashboard.rst`.
- Profiling how-to → `docs/onboarding/profiling.md` (6.20 variant,
  WP-PG.03).
- Compatibility decisions (D1–D8 outcomes) → `docs/onboarding/`
  release-management/compatibility notes.
- Per-PR evidence tables → PR bodies + CHANGELOG entries.
- Collision-backend lifecycle decisions →
  `docs/design/dart6_collision_backends.md`.
- Prior-art verdicts on the `origin/perf/dart6-*` experiment branches →
  recorded in the issue or onboarding notes; dead branches deleted from
  origin with maintainer approval.
