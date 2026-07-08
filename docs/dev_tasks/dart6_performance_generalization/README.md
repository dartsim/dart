# DART 6 performance generalization (issue #3056, round 2)

> **Status: ACTIVE — planning approved, packets open.** Single source of truth
> for the second round of DART 6.20 performance work on
> [issue #3056](https://github.com/dartsim/dart/issues/3056) (reopened
> 2026-06-30). Round 1 (#3123..#3203, closed out by #3199) solved the
> mostly-resting 3k-shapes regime; this round targets the **general active
> contact-rich regime**, the **gz-default ODE backend**, the **penetration-creep
> root cause that prevents piles from sleeping**, and **large-island solver
> scalability**, while coordinating with the native collision port.
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

Round 1 made settled scenes fast (3k_shapes DART-native RTF ~81, hash
`0x131b6af79a44ff90`, 3003/3003 resting; ODE RTF 0.05 → 18.1). What it did
not claim, and what the #3209 benchmark + PR body make measurable and
explicit, is the **active** regime and why scenes stay active:

- Active 120-body container: DART-native RTF ≈ 0.036, ODE ≈ 0.021; 16
  threads change almost nothing (see
  [01-baseline-evidence.md](01-baseline-evidence.md)).
- Profile smoke (active 60-body, native): the Dantzig LCP solve scope ≈ 66%
  of step time, collision ≈ 8%. Caveat: this maps to the solver's own
  profile scope; WP-PG.01/WP-PG.10 must record the `Construct LCP` vs
  solve-proper split before WS-A effort is committed.
- **Penetration creep blocks sleeping** (#3209 root-cause finding 2):
  contact error correction is capped at `DART_MAX_ERV = 1e-3` m/s
  (`ContactConstraint.cpp:49`); piles compact faster than the cap corrects,
  so max penetration grows ~5–10 mm/s without bound — and the island-rest
  veto requires every contact ≤ 1e-5 m penetration
  (`kSleepContactPenetrationTolerance`, `ConstraintSolver.cpp:2050`,
  `World.cpp:1229`), so **creeping piles can never freeze**. Detector- and
  solver-independent; deterministic reproducer recorded in
  01-baseline-evidence.md (scene S6).
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
| WS-F | Native collision port | [../dart6_dependency_minimization/03-native-collision-port-scoping.md](../dart6_dependency_minimization/03-native-collision-port-scoping.md) | phases 0–7 | **External owner** — 8-phase port plan (#3234). Tracked here for sequencing only; do not duplicate its packets |

Status across lanes lives in
[07-orchestration-dashboard.md](07-orchestration-dashboard.md).
Session handoff lives in [RESUME.md](RESUME.md). Prior experiments
(including measured-and-rejected approaches on the `origin/perf/dart6-*`
branches) are inventoried in 01-baseline-evidence.md — **read before
claiming any packet that overlaps them**.

## Success criteria (provisional until WP-PG.01 re-baselines; maintainer ratifies)

1. **Primary fixture** (active container, native detector, 1 thread,
   120 objects): cumulative default-on packets reach **RTF ≥ 3× the round-2
   baseline** (0.036 → ≥ 0.108) with all guard hashes stable.
2. **Pile-sleep outcome** (the gz-visible #3056 closer, needs D7/WP-PG.15):
   the S6 creep reproducer (71-object container, 20 s) ends with bounded
   `max_penetration` and all bodies resting under default settings.
3. **No regressions**: settled 3k native stays ≥ round-2 baseline RTF
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
  resting counts vs the recorded baseline for **every** detector (DART
  native, FCL, Bullet, ODE) — untouched backends must be bit-identical.
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
- Packet metadata: compiler, CPU/governor, pixi env, exact commands,
  which optional detectors were built (per the native-port scoping doc).
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
Decision-gated:               WP-PG.04 (D4), WP-PG.14 (D3), WP-PG.15 (D7),
                              WP-PG.23 (D8), WP-PG.33 (PG.30; D1/D2 for the
                              SIMD variant), WP-PG.42 (PG.40 + WS-F coord)
WS-F (native port) runs its own phases 0–7; its phase 4 consumes WS-D
kernels; WS-B depth is re-reviewed at its phase 5/6 (D5).
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
- Porting DART 7 world plumbing (EnTT) — see native-port scoping doc.
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
- **D3 — Matrix-free large-island solver**: opt-in solver option vs
  island-size-gated default. Proposal: opt-in on DART 6 (WP-PG.14).
  **Revisit trigger**: if WP-PG.10 confirms solve-proper dominance on
  single-connected-component islands, opt-in leaves the primary fixture
  without a default-on solve-side remedy — D3 must then be re-decided
  with that evidence.
- **D4 — Executor tooling**: keep `/dart-resume` + RESUME.md as the 6.20
  entry point, or backport a 6.20-adapted `dart-execute-packet` command
  (WP-PG.04).
- **D5 — ODE lane depth**: proposal — land the bounded set WP-PG.20/21/22
  (bookkeeping/algorithmic, behavior-preserving) and stop there pending
  WS-F phase 5 (facade decision). WP-PG.23 is governed by D8, not D5.
- **D6 — Deactivation default divergence**: 6.20 defaults deactivation ON,
  main defaults OFF. Flagged for awareness; no action proposed this round.
- **D7 — Penetration-creep remediation policy** (blocks WP-PG.15): the fix
  space spans the contact ERV budget (`DART_MAX_ERV`; a public setter
  `ContactConstraint::setMaxErrorReductionVelocity` already exists),
  island-LCP convergence for large piles, and/or a bounded-penetration
  tolerance in the island-rest veto. All are behavior-changing (contact
  depths, sleep transitions); decide the acceptance envelope and
  re-baseline process before implementation.
- **D8 — Contact-manifold reduction on current detectors** (blocks
  WP-PG.23): round 1 explicitly deferred default-on manifold
  reduction/selection ("mines #2366 and DART 7 native collision"; recorded
  FCL evidence: 2.6× RTF from per-pair capping). Options: pursue now as a
  behavior-changing packet on the ODE/FCL wrappers, or defer to WS-F
  phase 3 (native engine) with the risk that WS-F slips. Deciding *not*
  to do it now must be recorded as a decision, not an omission.

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
- Anything native-collision → the dep-min scoping doc (its owner).
- Prior-art verdicts on the `origin/perf/dart6-*` experiment branches →
  recorded in the issue or onboarding notes; dead branches deleted from
  origin with maintainer approval.
