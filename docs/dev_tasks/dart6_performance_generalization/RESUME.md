# RESUME — DART 6 performance generalization

Read [README.md](README.md) first (envelope, gates, success criteria,
decisions D1–D8), then the lane doc of the packet you claim, then update
[07-orchestration-dashboard.md](07-orchestration-dashboard.md). Read the
prior-art inventory in
[01-baseline-evidence.md](01-baseline-evidence.md) before claiming any
packet that overlaps the `origin/perf/dart6-*` experiment branches.

## Next packets

**2026-07-31: current-base re-baseline RAN on `718651d0d6e`** (session
branch `wp-pg-wsg-rebaseline-20260731`; guard artifacts
`/tmp/wsg_rebaseline_guards_20260731`, A/Bs `/tmp/wsg_ab_20260731`,
WS-G rerun `/tmp/mj_cmp_20260731`). Read the new
"2026-07-31 current-base guard refresh" section of
[01-baseline-evidence.md](01-baseline-evidence.md) first. Summary:

- Criterion 1 (S1 primary fixture): **MET, improved** — same-host
  interleaved A/B shows the current stack 1.29x faster than the
  audit-head stack on S1 120/dart/1 (chained ≈4.5x over the round-2
  baseline). July step-time cells are not comparable to today's host
  clock state; an apparent 1.6–6x slowdown was a host artifact, refuted
  by ABAB A/Bs with bit-identical per-arm hashes.
- Criterion 2 (S6 pile-sleep): **REGRESSED — the selected gap.** The
  #3381 consolidated `dart` detector's box-box stream (≤3-point
  manifolds, pair churn, penetration plateau ~4.4 mm) keeps the pile
  above the wake band forever; 0/71 resting under defaults. FCL control,
  audit-head binary, and pre-#3381 binary all still sleep 71/71 (the
  latter two bit-exact `0xec80f734df6d5e74`), isolating the #3381
  detector swap as the sole cause. Policy knobs do not rescue it.
- Criterion 3 (no regressions elsewhere): fcl/bullet/ode guard rows held
  bit-identical (or were re-established where July references were
  lost); all `dart` rows re-baselined per the #3381 PR body's
  breaking-changes note ("its contact profile can differ"); S2/S3 dart
  now coincide with FCL fixed points.
- WS-G: the full 8-scene matrix (both HUM rows included) ran on the
  merged base with provenance, plus a post-fix box-row rerun; standings
  live in [08-mujoco-comparison-lane.md](08-mujoco-comparison-lane.md).

Implementation packet executed by this session (WP-PG.50 candidate,
checkpoint commit `03046383e77` on `wp-pg-wsg-rebaseline-20260731`;
note: the first two commits were pushed to origin on 2026-08-01 outside
the session — the later commits remain local): the solver-facing
3-contact clamp in `DARTCollisionDetector.cpp` was raised to the full
4-contact manifold capacity, with pinned regression tests and a
CHANGELOG entry (`#PENDING` link — fill at PR time). Full evidence and
the measured trade-off live in the "2026-07-31 manifold fix" section of
[01-baseline-evidence.md](01-baseline-evidence.md): resting scenes
improve up to 4x and FCL/Bullet/ODE stay bit-identical, but the
always-active dense fixture pays ~2x (criterion 1 falls to ≈2.3x,
below the 3x bar), and S6 improves (5/71 at 20k, dwell accumulating)
without fully sleeping — the residual blocker is contact-stream
persistence (pair churn), not sleep policy (a tolerance raise was
measured and rejected: it triggers freeze-under-load penetration
explosions). 154/154 C++ tests pass; lint/check-lint clean.

**2026-08-01 final — D9 and D10 DECIDED and criterion 2 MET (the
maintainer delegated both calls in-session on 2026-08-01 with a
root-cause, evidence-based, A/B-verified mandate; full records in README
"Open decisions").** The session measured old-vs-new contact streams on
constructed micro-pose sweeps, then fixed four detector stream defects
as one bundle: the 3-contact solver-facing clamp, wandering cylinder
side-line points (2-point line manifolds + effective-radius correction),
intermittently missed crossed-cylinder contacts, and — the criterion-2
closer — spin-variant aligned cylinder-box handling (an upright cylinder
with arbitrary spin fell off the stable cap-patch path onto degenerate
convex rim points and crept without bound; found via a 60k trend audit
+ final-scene reconstruction, fixed by spin/permutation
canonicalization). Final state: S6 sleeps **71/71 with penetration 0**
on 60k runs of the canonical seed and the worst creeper; 4/5 seeds
fully deactivate within the original 20k window (audit-era stack: 3/5);
the bundle is **0.946x vs the audited pre-consolidation stack**
(slightly faster) on the direct S1 interleave; 154/154 C++ tests,
cylinder suite 32/32; S2-S5 dart guard hashes stable across the last
two fix iterations; S1 re-baselined
(`120: 290/178 0xfc20c4880fdbca05`, `60: 88/74 0x6dab35ce2618d422`).
Codex provided the independent cylinder root-cause confirmation and the
code-review lane whose effective-radius finding was verified and
applied. Work is preserved as LOCAL commits on
`wp-pg-wsg-rebaseline-20260731` (never pushed; push/PR needs explicit
approval); raw artifacts and scripts are archived at
`~/dart-wsg-evidence-20260731/` including the `aug01/` tree. Next
action: shepherd PR #3428 through CI and review per `dart-manage-pr`
(no thread replies to AI reviewers; local fixes + re-review requests
only with approval), then retire this folder in the completing PR per
the closeout plan.

**2026-07-10: the current-head completion audit RAN** (release-6.20 @
`db255a08e8e`; artifacts `/tmp/audit_head_20260710T011207Z`):

- Criterion 1 (S1 primary fixture 3x): **MET** — 120/dart/1 RTF 0.126447
  (3.51x the 0.036 round-2 baseline), avg step 7.908 ms, hash
  `0x123ee9779bccacfb`.
- Criterion 2 (S6 pile-sleep): **MET** — 71/71 resting, max_pen 0, hash
  `0xec80f734df6d5e74` (exact #3353 accepted value), RTF 0.21.
- Criterion 3 (no regressions): all S2/S3 guard hashes bit-identical across
  dart/fcl/bullet/ode; S4/S5 fcl match the recorded drift values; S4/S5
  bullet re-baselined by merged #3355's analytic PlaneShape path (new values
  in the audit dir are the current guards).
- Criterion 4 (general evidence): the maintainer broadened the bar to
  cross-engine evidence vs MuJoCo across DART's major workloads. New lane
  **WS-G**: [08-mujoco-comparison-lane.md](08-mujoco-comparison-lane.md).
  Harness + mujoco pixi env + compiled dartpy binding: PR #3367. The earlier
  prototype ant result is deferred because its exact command, artifact, and
  engine SHAs were not retained; do not count it as accepted cross-engine
  evidence. Reproducible standings, gaps, and root causes are recorded in the
  lane doc with packets. #3369 merged the MJCF stacked-joint and collision
  fidelity work; the WP-SS small-scene-overhead family remains.
- Side products: #3366 fixes a dartpy `getDofs`/`getChainDofs` ownership bug
  (heap corruption, SIGSEGV at teardown); #3368 (dep-min lane) removes the
  `dart` detector's O(n^2) broadphase with bit-identical guards.

#3366, #3367, #3368, and #3369 have merged; docs refresh is this PR. Re-baseline
the WS-G and `dart` detector rows on the current merged base before cutting
the next evidence-driven packet.

A fresh session should start from current `origin/release-6.20` (the audited
head above or later; re-fetch — the maintainer merges frequently), read this
README plus the lane docs, and continue from the WS-G lane on the merged base
rather than redoing the completed audit. The maintainer's
north-star requirement is broader than "latest packet merged": finish issue
#3056 on DART 6.20 with cross-engine evidence (WS-G) showing the result is
general. The whole remaining effort should land in a few large, cohesive PRs.

Do not open a small follow-up PR merely because a packet exists. Prefer one
consolidated evidence/closeout branch unless the audit identifies a real
implementation gap that deserves its own packet.

WP-PG.14/D3 is done by #3361. It added a default-off
`BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions` path, exact
built-in `ContactConstraint` gating, single-free-body support checks,
prepared scratch reuse, cached-impulse residual seeding, dense fallback on
matrix-free non-convergence, mixed per-DoF FreeJoint actuator rejection,
`contact_benchmark` flags, dartpy options, and profiler counters. Final
current-head smoke artifact:
`/tmp/wp_pg14_matrix_free_review_5751c7ed84c_repeat_20260709T223525Z`
(`5751c7ed84c`): S1 120 DART dense option-off median avg step `7.93478` ms,
hash `0x123ee9779bccacfb`; option-on 30-iteration matrix-free median avg step
`1.38782` ms, finite, hash `0xa5548e1abe05b52` (~5.7x on that run). The
original fuller artifact remains
`/tmp/wp_pg14_matrix_free_ab_20260709T040443Z` with S3 active-3k option-on
fallback preserving hash `0xcf0ba6eaa97be038`.

Recommended next session plan (items 1-2 are DONE for the 2026-07-10 cycle;
kept for the method):

1. Verify live state: `git fetch origin release-6.20`, inspect open
   PRs/issues, and avoid touching dirty sibling worktrees
   such as `/home/js/dev/dartsim/dart/task_3-fix-simd`.
2. Run a current-head acceptance audit against the README north-star gate:
   tests, benchmark matrix, GUI/headless artifacts, decision status, and
   remaining issue #3056 closure evidence.
3. Refresh cumulative benchmark evidence on the current merged head, not on
   individual PR heads. Include the issue-specific 3k/gz default ODE path,
   active contact-container rows across relevant detectors, S6/default sleep
   evidence, settled-scene guard rows, and option-off/option-on semantics where
   WP-PG.14 is discussed.
4. Refresh representative GUI/headless evidence if current artifacts are not
   enough for closeout: `contact_benchmark --gui-capture`, sleep-state colors
   where useful, and final-scene dumps with command lines and hashes.
5. Decide from that evidence whether remaining work is a code packet
   (for example D8/WP-PG.23, WP-PG.33, or WP-PG.41) or a closeout/promote-docs
   PR. Do not revive WP-PG.11/21/22 unchanged; each already has current-base
   rejection evidence below.

WP-PG.32 is closed in this tracker as delivered by merged PRs #3297 and #3307:
`FrameAllocator`, World-owned `MemoryManager` preparation, frame-scratch
capacity/overflow counters, `FrameStlAllocator`, DART-owned
solver/collision/profiler scratch reuse, and `INTEGRATION_StepAllocation`
`dart` detector and soft allocation gates are already on `release-6.20`. The
`wp-pg-32-frame-allocation-gate` branch should not be resumed as a new
implementation packet after its PR lands.

**WP-PG.01 is captured** (merged as #3263) — guard rows, profile splits, and
prior-art triage are in
[01-baseline-evidence.md](01-baseline-evidence.md). Its evidence changed
priorities: the dense-pile fixture is 88.1% Dantzig solve-proper (D3
revisit trigger FIRED; WP-PG.12 deprioritized), and the active-3k
many-islands regime is ~50% integration (WS-C is the lever there).

Implementation packets after WP-PG.32, in priority order:

1. Run the current-head completion/evidence audit above before choosing more
   implementation.
2. Continue with the remaining north-star evidence and any still open D8/D4
   decisions rather than treating WP-PG.14/#3361 as task completion.
3. Do not retry rejected packets unchanged. WP-PG.20 is #3329; WP-PG.21 failed
   the 2026-07-07 current-base map/pruning gate; WP-PG.22 and WP-PG.11 both
   have local current-base rejection evidence from 2026-07-06; WP-PG.31 is
   #3341; WP-PG.32 is already implemented by #3297/#3307.

Blocked/gated (do not claim without new evidence): PG.04 (D4),
PG.12 (evidence), PG.13 (PG.10 census), PG.23 (D8), PG.33, PG.41.
PG.15/D7 is done as merged #3353; PG.14/D3 is done as merged #3361. PG.42 is
done in PR #3299; WP-PG.30 is done in PR #3310. The dense-pile fixture now has
the D7 default-remediation win; WP-PG.14 provides the separate explicit
solve-side option for large active islands.

## Verify commands (every packet)

```bash
pixi run lint          # auto-fix
pixi run check-lint    # verify — matches CI's Check Lint
pixi run cmake --build build/default/cpp/Release --target ALL --parallel 8   # capped build
DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz   # collision/constraint/solver/header packets
pixi run test-eigen-overalignment                 # allocation/alignment packets
# Determinism guard + benchmark evidence: see 01-baseline-evidence.md
```

(`DART_PARALLEL_JOBS` is honored by the gz and eigen-overalignment
scripts; `pixi run test-all` runs unbounded `cmake --build ... -j`, hence
the explicit `--parallel 8` build command.)

## Standing decisions awaiting maintainer

D1 (SIMD FP contract), D2 (ISA delivery), D4 (executor tooling), D5 (ODE lane
depth = PG.20/21/22), D8 (manifold reduction now vs WS-F phase 3) — see
README "Open decisions". D7 is resolved by merged #3353. D3 is resolved by
merged #3361 as a DART 6 default-off matrix-free contact solver option; do not
reopen it as a default-on behavior without new maintainer direction and
option-off/option-on evidence.

## Session log (round-2 execution)

- 2026-08-01 (later): The evidence audit caught "bounded penetration"
  over-asserted — 60k trends showed seed-101 penetration GROWING
  0.100 → 0.137 m. Final-scene reconstruction identified an upright
  spun cylinder creeping through its cap support: the aligned cap-patch
  path demanded exact rotational identity, so spun uprights fell to
  degenerate convex rim points. Fixed by spin/permutation
  canonicalization of the aligned cylinder-box path (+ side-line hoist
  with a shallow-pose gate); four stale single-support test pins
  modernized to line manifolds; spin-invariance pinned. Outcome: S6
  71/71 pen 0 at 60k on both probe seeds, 4/5 seeds sleeping within
  20k, parity interleave 0.946x (bundle faster than the audited stack)
  — criterion 2 met on original terms, D10's re-anchoring superseded.
- 2026-08-01: Maintainer delegated D9/D10 with a root-cause,
  evidence-based mandate. Stream-quality probe suite (micro-pose sweeps
  on constructed states, old vs new dartpy builds) showed parity or
  new-better everywhere except two cylinder defects: a side-on-face line
  contact wandering ~90 mm at every tilt (both engines; GJK fallback
  root cause, independently confirmed by a Codex lane) and new-only
  intermittent misses of shallow crossed-cylinder contacts. Implemented
  `tryAddCylinderBoxSideLineContacts` (clipped two-point line manifold,
  per-endpoint depths) and `tryAddCrossedCylinderSideContact`
  (capsule-equivalent interior closest-point contact) with three pinned
  tests; cylinder suite 30/30, full suite 154/154, untouched detectors
  bit-identical, S2/S3 dart hashes unchanged, S1 re-baselined. Direct
  quiet-host interleave vs the audit stack: **1.009x parity** (the
  earlier ~2x was the manifold-only intermediate) → criterion 1 ≈3.5x
  MET → D9 = SHIP the bundle. 5-seed S6 matrix on three stacks proved
  the mixed-pile all-resting outcome seed-chaotic everywhere (audit
  stack 3/5, bundle ~1.5/5, bounded penetration universal); a
  dartpy-authored box-only pile converges ~100x quieter than the mixed
  pile but its island freeze does not latch in 40k steps → D10 =
  re-anchor criterion 2 (bounded penetration + pinned stream tests) with
  bounded follow-ups (deactivation latch, rolling friction,
  parallel-line 1-point contacts). Codex was available again and used
  for the cylinder diagnosis lane.
- 2026-07-31: Full re-baseline on `718651d0d6e` (branch
  `wp-pg-wsg-rebaseline-20260731`). S1–S6 guard matrix re-established with
  drift classification (fcl/bullet/ode held bit-identical; all `dart` rows
  re-baselined per the #3381 PR body's contact-profile note; S2/S3 dart land
  on FCL fixed points). Criterion-2 regression found and root-caused: the
  consolidated detector's solver-facing 3-contact clamp breaks resting
  face-face support; attribution chain closed by bit-exact S6 reproduction
  (`0xec80f734df6d5e74`) on both the audit-head and pre-#3381 binaries and
  an FCL control that sleeps 71/71 on the current base. An apparent broad
  wall-time regression was refuted as host clock-state artifact via
  interleaved A/Bs (hashes bit-identical per arm). First complete 8-scene
  WS-G matrix ran (HUM rows first-ever; 3 wins / 4 losses pre-fix).
  WP-PG.50 candidate implemented: solver-facing manifold clamp 3 → 4 with
  pinned tests (154/154), changelog draft, S6 GUI capture, and a measured
  trade-off (resting scenes 4x faster, ARM-PUSHER flips to a win; dense
  always-active fixtures ~2x slower, DYN-STIR flips to a loss; S6 partial
  recovery only). Tolerance-raise alternative measured and rejected
  (freeze-under-load penetration explosion). Ship/hold gated on D9; S6
  stream-persistence follow-up scoped as D10. Review pass 1 (fresh-context
  correctness lane) returned no blockers; findings applied. Codex was
  unavailable this session (weekly limit), so review lanes are
  role-separated local subagents — recorded as a limitation.

- 2026-07-04/05: WP-PG.01 executed on `wp-pg-01-baseline-evidence`:
  original matrix/profile/dashboard capture on `origin/release-6.20`
  @ `5bee91ad6be`, then current-base guard refresh on
  `origin/release-6.20` @ `b9e6910c066`. Full S1–S6 guard matrix is
  recorded in 01-baseline-evidence.md; profile splits recorded (P1
  dense-pile: solve 88.1% / assembly 7.4%; P2 active-3k: integration
  50.1%); six prior-art branches triaged (two unmerged hash-preserving
  solver wins queued into WP-PG.11; one branch recommended for deletion).
- 2026-07-06: Earlier local WP-PG.20 span/no-copy variants were rejected
  against then-current `release-6.20` because the weakest 120-object/
  16-thread ODE row regressed; those probes are superseded by the corrected
  #3329 implementation below.
- 2026-07-06/07: WP-PG.20 executed on #3329
  `wp-pg-20-ode-history-spans`: ODE contact-history spans without the
  pair-keyed map/pruning follow-up. After the #3307/#3327 base advance,
  current-base A/B on `origin/release-6.20` @ `9ff8b1d77a1` kept hashes
  bit-identical and improved ODE rows: `S2_ode` 0.0933 -> 0.0521 ms/step,
  `S3_ode` 115.9 -> 19.7 ms/step, `S4_ode` 0.2269 -> 0.1549 ms/step,
  and `BM_ContactContainerActive` ODE rows by 3.2-7.6%. WP-PG.21 remains
  open but evidence-gated against the span-only baseline.
- 2026-07-06: WP-PG.22 was attempted locally but not published: a safe
  exact-transform ODE pose-write gate preserved hashes but regressed the
  settled 3k/900-object ODE rows; the intended cpp-only kinematic-version
  route is blocked because `Skeleton::getKinematicVersion()` is protected
  on this branch. Work moved to WP-PG.11 on
  `wp-pg-11-solver-rtti-scans`.
- 2026-07-06: WP-PG.11 mined the two round-1 single-reactive solver
  commits locally, but was reverted and not published after `release-6.20`
  advanced to `2e11928288c`: refreshed A/B medians regressed S2 ODE 3k
  settled (0.87x), S4 generated-900 DART (0.93x), S4 Bullet (0.99x), and
  S4 ODE (0.98x). FCL and active S1 ODE improved, but the required
  general-performance bar was not met; all compared rows had identical
  contacts, pairs, resting counts, and final hashes. Artifacts:
  `/tmp/wp_pg11_ab/current_2e119_repeat`. Work moved to WP-PG.02 to
  strengthen benchmark coverage before more optimization packets.
- 2026-07-06: WP-PG.02 claimed on
  `wp-pg-02-contact-container-matrix`. Active contact-container rows now
  preserve the old DART/ODE 60/120 rows and add FCL/Bullet plus a
  4-thread sweep. Separate bounded DART/ODE deactivation-enabled rows are
  included in the dashboard filter; dense 900-object contact-container
  rows are registered for manual filters but kept out of the default
  dashboard slice after local smoke runs exceeded the runtime budget.
  Artifacts: `/tmp/wp_pg02_contact_container_deactivation_rows.json`,
  `/tmp/wp_pg02_contact_container_active_fcl_bullet_smoke.json`.
- 2026-07-07: WP-PG.02 merged as #3327. WP-PG.03 claimed on
  `wp-pg-03-profiling-doc` to promote the DART 6.20 profiling workflow into
  `docs/onboarding/profiling.md` and add the profile-env Tracy configure task.
  End-to-end profile build exposed an existing `TRACY_CALLSTACK` compatibility
  bug in `dart/common/Profile.hpp`; the branch fixes it by using Tracy's
  callstack constructor only when the packaged header exposes the callstack
  macro. Local verification passed: `pixi run -e profile config-tracy`, profile
  `contact_benchmark` build + one-step `--profile` smoke,
  `UNIT_common_Profile`, docs parse smoke, `pixi run lint`, and
  `pixi run check-lint`.
- 2026-07-07: WP-PG.03 merged as #3337. A fresh WP-PG.21 gate on current
  `origin/release-6.20` @ `b78a8b8cbe7` re-tested the prior ODE
  pair-keyed map/pruning variant against the span-only #3329 base. Hashes
  stayed identical, but the result was mixed/regressive: `S2_ode` 0.0572 ->
  0.0644 ms/step (+12.6%), `S3_ode` 22.28 -> 19.74 ms/step (-11.4%),
  `S4_ode` 0.208 -> 0.248 ms/step (+19.2%), and contact-container 120-object
  ODE rows regressed 6.8-9.4%. Artifact:
  `/tmp/wp_pg21_gate_20260707T130843`; WP-PG.21 remains evidence-gated.
- 2026-07-07: WP-PG.10 claimed on `wp-pg-10-lcp-profile-census`. The branch
  adds text-profiler counters and scopes for constrained-group island census
  and boxed-LCP stage split, plus runtime-gated solver recording for
  allocation-sensitive non-profile runs. Local profile artifact:
  `/tmp/wp_pg10_profile_20260707T132241`; local guard artifact:
  `/tmp/wp_pg10_guard_20260707T132321`. DART S1-S5 hashes matched the guard
  references; S1 120-object dart/ode rows and S2-S5 all-detector rows matched
  current-base parent. The old S4/S5 FCL guard hashes had already drifted on
  unmodified `origin/release-6.20` (`S4_fcl = 0xea9b68f8b062600d`,
  `S5_fcl = 0x8277be4f0c14212`), so they are not WP-PG.10 regressions.
  Final verification passed lint/check-lint, capped `ALL` (144/144 C++ tests,
  98/98 Python tests), profile smoke, allocation gates, and gz-physics/gz-sim
  compatibility (`test-gz`).
- 2026-07-08: WP-PG.10 merged as #3339 after Codex review reported no major
  issues on `d099efa2c9`. WP-PG.31 claimed on
  `wp-pg-31-shallow-support-scratch`.
- 2026-07-08: WP-PG.31 implemented as the no-candidate shallow-support skip on
  current base `3964108a675` (branch `21f691311df`), because current
  `World.cpp` already retained the scratch buffers from the old preserved
  patch. A/B artifact `/tmp/wp_pg31_ab_20260707T184319`: no-root-FreeJoint
  `double_pendulum.world` hashes stayed `0x1db838038acbd960` with median step
  time 0.002106 -> 0.001836 ms (DART) and 0.001903 -> 0.001644 ms (ODE);
  generated 120-object DART/ODE guard hashes stayed
  `0x3bd9d26da5ea002b` and `0x5576f6244b736aae`; ODE
  `BM_ContactContainerActive/120/1/{1,16}` medians were 6588 -> 6433 ms and
  6998 -> 6468 ms. Verified binaries passed the historical crash gate: base
  30/30 and branch 30/30 default 16-thread benchmark stressors returned 0.
- 2026-07-08: WP-PG.31 merged as #3341 (`653da1256fb`). WP-PG.32 claimed on
  `wp-pg-32-frame-allocation-gate` as a tracker reconciliation: merged #3297
  (`93afb066c66`) had already added `FrameAllocator`, World-owned
  `MemoryManager` preparation, solver/profiler/Dantzig scratch reuse, and the
  `INTEGRATION_StepAllocation` allocation-counting gate; merged #3307
  (`b6e6a0d8778`) extended the same allocation discipline through soft-body
  paths and recorded the full #3307-style performance report. Current local
  verification is recorded in the WP-PG.32 row of the dashboard and lane doc.
- 2026-07-08: The attempt to retire this task in #3353 was reversed after the
  maintainer clarified the north-star goal: fully complete the performance
  optimization dev task for #3056 with broad tests, benchmarks, and GUI/example
  evidence. #3353 is now the WP-PG.15/D7 evaluator PR. Local evidence so far:
  `contact_benchmark` exposes `--contact-max-erv` and
  `--sleep-contact-penetration-tolerance`; focused
  `test_IslandDeactivation` guards passed; short S6 sweeps showed ERV reduces
  penetration but does not make the pile sleep by itself, and a 20 s S6 run
  with `--contact-max-erv 0.1 --sleep-contact-penetration-tolerance 0.005`
  stayed finite with bounded max penetration (~0.0037 m) but still ended
  0/71 resting because the dense-pile velocity jitter remained above the
  final-quiet gate. At that point #3353 was evaluator-only; the later
  default-remediation evidence below supersedes this note.
- 2026-07-08: #3353 was advanced from evaluator-only to the D7
  default-remediation implementation after A/B evidence showed threshold
  widening alone was insufficient and broad global ERV regressed simple
  support-contact tests. The candidate promotes the static contact ERV default
  `0.001 -> 0.1` but applies the higher effective ERV only to dense islands
  with mobile-mobile contacts; single-mobile static-support islands keep the
  legacy effective cap. Dense islands also get a `0.005` solver rest-veto
  tolerance under the default policy, plus dense-contact-island sleep candidacy
  for sub-wake jitter. `contact_benchmark` now reports island and
  dwell/velocity diagnostics. A/B artifact
  `/tmp/wp_pg15_ab_plane_fallback_20260709T023141Z`: S6 old-default override
  (`--contact-max-erv 0.001 --sleep-contact-penetration-tolerance 0.00001`)
  took 212.08 s, RTF 0.0943043, 162 contacts / 141 pairs, max penetration
  0.364241, 0/71 resting, hash `0x159825257114c5d5`; current defaults took
  91.9572 s, RTF 0.217493, zero contacts, max penetration 0, 71/71 resting,
  hash `0xec80f734df6d5e74`; the explicit global evaluator row (`ERV=0.1`,
  tol `0.005`) took 75.487 s, RTF 0.264946, zero contacts, 71/71 resting,
  hash `0x877687e64e1011b9`. S4/S5 DART/FCL/Bullet/ODE new-default rows in
  `/tmp/wp_pg15_ab_review_20260708T235540Z` matched old-default hashes,
  contacts, and resting states. Extra evidence:
  S2 DART 3k-shapes guard
  `/tmp/wp_pg15_examples_20260708T223506Z/S2_dart_3k_shapes.log`
  (`3003/3003` resting, hash `0x8ddc9a81f2d28a7f`), S6 final-scene dump
  `/tmp/wp_pg15_visual_20260708T223506Z/S6_final_scene.jsonl`, and S6 GUI
  capture `/tmp/wp_pg15_gui_20260708T223653Z/S6_gui.png` with a passing
  non-blank `image-verdict`.
  Focused tests passed: `INTEGRATION_StepAllocation` DART allocation gates,
  `test_SplitImpulse` shallow-support guards, `test_Issue1445`, full
  `test_IslandDeactivation`, and full `DARTCollisionDetector.*`.
  CI failures on the old PR head were traced to `test_DARTCollisionDetector`
  helper filters needing `final` under clang/libc++ warning-as-error builds;
  that test-only fix landed through the refreshed base.
- 2026-07-08: An attempted overhead trim for the dense-island World sleep
  candidacy path was rejected and reverted after an A/B rerun
  (`/tmp/wp_pg15_ab_candidate_20260708T224930Z`) changed the S6 final hashes
  and made the explicit evaluator row slower (82.9871 s vs the prior
  38.9195 s) without a clear S4/S5 guard-row timing win. Keep the
  island-atomic candidate path from the accepted candidate unless a future
  optimization carries a cleaner old/new matrix.
- 2026-07-08: After merging `origin/release-6.20` through #3355, capped `ALL`
  first failed `SdfParser.PlaneShapeBulletWorldSettles`: the new Bullet
  analytic `PlaneShape` path converged with millimeter-scale penetration and
  never satisfied the strict `1e-5` rest-veto tolerance. The fix keeps small
  non-plane support islands on their legacy individual candidacy path, keeps
  dense islands on the atomic sub-wake path, and applies the bounded `0.005`
  rest-veto tolerance only to dense islands and analytic `PlaneShape` contacts.
  Focused rechecks passed: `test_SdfParser
  --gtest_filter='SdfParser.PlaneShapeBulletWorldSettles' --gtest_repeat=3`,
  `INTEGRATION_StepAllocation` DART allocation gates, `test_SplitImpulse`
  shallow-support guards, `test_Issue1445`, and full `test_IslandDeactivation`.
- Final pre-push gates for the current candidate passed locally after the Codex
  review fix and public-query cleanup: `pixi run lint`,
  `pixi run check-lint`, capped `pixi run cmake --build
  build/default/cpp/Release --target ALL --parallel 8` (98 Python tests and
  148 CTest tests), and `pixi run -e gazebo test-gz` (`gz-physics` 199/199
  plus 4 performance tests; `gz-sim` `INTEGRATION_entity_system` passed against
  the source-built DART plugin).
- 2026-07-08 Codex review fix: explicit default-valued
  `setAutomaticSleepingContactPenetrationTolerance(1e-5)` calls now preserve
  the strict legacy rest-veto policy instead of being treated as the implicit
  adaptive default. Added
  `IslandDeactivation.ExplicitDefaultToleranceKeepsPlaneContactStrict` and
  regenerated the A/B matrix above; the strict old-default S6 row now takes
  197.777 s and never reaches resting, while current defaults keep the accepted
  71/71 resting hash.
- 2026-07-08 Codex review fix: the ungrouped-awake-body veto now clears the
  group-level rest flag before `solveConstrainedGroups()`, so a newly eligible
  contact island cannot be marked resting by the final solve while another
  mobile body is still awake. Added
  `IslandDeactivation.UngroupedAwakeBodyVetoesNewContactIslandResting` and
  reran the S6 A/B rows in
  `/tmp/wp_pg15_ab_awake_veto_20260709T005203Z`; accepted-default and
  strict-old hashes stayed unchanged.
- 2026-07-08 Codex review fix: temporary contact ERV overrides can now return
  to the adaptive default policy with `resetMaxErrorReductionVelocity()`, while
  explicit default-valued setter calls remain idempotent broad overrides. Added
  `IslandDeactivation.DefaultContactErvRestoresAdaptivePolicy`; reran the
  current-head S6 A/B rows in
  `/tmp/wp_pg15_ab_idempotent_20260709T015150Z`; accepted-default,
  strict-old, and explicit-evaluator hashes stayed unchanged.
- 2026-07-08 Codex review fix: the contact-miss sleep fallback now applies the
  same default adaptive `PlaneShape` tolerance as the constrained-group rest
  gate, while explicit strict tolerance overrides still clear unislanded sleep
  candidates. Added
  `IslandDeactivation.PlaneContactMissFallbackUsesAdaptiveDefaultTolerance`;
  reran the S6 A/B rows in
  `/tmp/wp_pg15_ab_plane_fallback_20260709T023141Z`; accepted-default,
  strict-old, and explicit-evaluator hashes stayed unchanged.
- 2026-07-09: #3353 merged after Codex review reported no major issues on
  `9ab3e05332`. Post-merge current-base checks on `origin/release-6.20` showed
  S6 current defaults still reached `71/71` resting with hash
  `0xec80f734df6d5e74`, and S1 120-object DART/ODE rows now exceed the
  original primary-fixture 3x target under default settings. WP-PG.14 was
  claimed on `wp-pg-14-matrix-free-lcp`. Local A/B artifact
  `/tmp/wp_pg14_matrix_free_ab_20260709T040443Z`: option-off S1 120 DART
  median avg-step `7.40446` ms, hash `0x123ee9779bccacfb`; option-on 30-iter
  matrix-free median avg-step `0.663138` ms, finite, hash
  `0xbf538ac9d35f145e`; S3 active-3k option-on fallback preserved hash
  `0xcf0ba6eaa97be038`. Local gates passed: `pixi run lint`,
  `pixi run check-lint`, focused/full `test_ConstraintSolver`, dartpy
  constraint pytest, capped `ALL`, and
  `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`.
- 2026-07-09: WP-PG.14 merged as #3361 after merge-conflict resolution,
  Codex review cleanup, local validation, and mostly passing hosted CI.
  Merge commit: `91c158fc3e5`; PR head before merge:
  `5751c7ed84c`. Review-hardening fixes added reserved thread-scratch
  `BodyNode*` lookup storage, cached-impulse residual seeding before the first
  matrix-free sweep, dense fallback when matrix-free PGS does not converge, and
  mixed per-DoF FreeJoint actuator rejection. Added regression coverage:
  `MatrixFreeContactSolverSeedsCachedImpulseResidual`,
  `MatrixFreeContactSolverFallsBackWhenNotConverged`, and
  `MatrixFreeContactSolverRejectsMixedFreeJointActuators`. Final current-head
  smoke artifact:
  `/tmp/wp_pg14_matrix_free_review_5751c7ed84c_repeat_20260709T223525Z`;
  S1 120 DART dense option-off median avg step `7.93478` ms, hash
  `0x123ee9779bccacfb`; option-on 30-iteration matrix-free median avg step
  `1.38782` ms, finite, hash `0xa5548e1abe05b52`. Final local validation:
  `pixi run lint`, `pixi run check-lint`, build target
  `contact_benchmark test_ConstraintSolver dartpy`, full
  `test_ConstraintSolver` (48/48), and dartpy constraint pytest (4/4).
  Codex reviewed `5751c7ed84c` and reported no major issues; all earlier
  Codex review threads were resolved. Hosted Codecov patch/project passed
  before merge.

## Session log

- 2026-07-04: WP-PG.30 executed (Codex worker + orchestrator review):
  version-keyed cached root-FreeJoint classification; four dynamic_cast
  sites removed; part (b) FD fast path analyzed and skipped with in-code
  rationale (recursion already minimal; round-1 landmine avoided).
  Adversarial review: 4 findings fixed (write ordering, zero-body
  contract, private section, getRootJoint reuse). Evidence: 8/8
  bit-identical hashes; win concentrated in many-islands regime.

- 2026-07-04: Folder created on branch `perf/dart6-3056-performance-plan`
  (planning PR). Investigation evidence: issue #3056 reopened 2026-06-30;
  round-1 stack inventory reconstructed; active-regime baseline probe and
  Dantzig-share profile smoke recorded in 01-baseline-evidence.md.
- 2026-07-04: Incorporated the #3209 root-cause findings (penetration
  creep vs island-rest veto → WP-PG.15/D7; ODE cylinder-fallback cap
  flood → WP-PG.23/D8 + ODE-row validity rule) and the round-1 experiment
  branch handoff (prior-art inventory + rejected-experiments list).
  Adversarial plan review applied (sequencing consistency, WP-PG.11
  re-scope, WP-PG.13 premise correction, success criteria added).
- 2026-07-05: WP-PG.40 standalone PR #3270 closed per maintainer
  direction. WP-PG.42 claimed on `wp-pg-42-soa-broadphase-simd`; live
  WS-F check found #3281 merged only internal collision math, so
  this branch remains the DART 6 detector SIMD-consumer path.
