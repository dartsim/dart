# RESUME — DART 6 performance generalization

Read [README.md](README.md) first (envelope, gates, success criteria,
decisions D1–D8), then the lane doc of the packet you claim, then update
[07-orchestration-dashboard.md](07-orchestration-dashboard.md). Read the
prior-art inventory in
[01-baseline-evidence.md](01-baseline-evidence.md) before claiming any
packet that overlaps the `origin/perf/dart6-*` experiment branches.

## Next packets

**Current claimed packet: WP-PG.03 — DART 6 profiling documentation + Tracy
config task** ([06-infra-evidence-lane.md](06-infra-evidence-lane.md)). Continue
on `wp-pg-03-profiling-doc` off current `origin/release-6.20`.

Immediate next step: open the packet PR after maintainer approval, then monitor
hosted CI. Local verification already passed (`pixi run -e profile
config-tracy`, profile `contact_benchmark` build + one-step `--profile` smoke,
`UNIT_common_Profile`, docs parse smoke, `pixi run lint`, and
`pixi run check-lint`). This packet is docs/config plus the small Tracy
callstack compatibility fix required to make the new profile task build; do not
include performance speedup claims.

**WP-PG.01 is captured** (branch `wp-pg-01-baseline-evidence`, PR
pending) — guard rows, profile splits, and prior-art triage are in
[01-baseline-evidence.md](01-baseline-evidence.md). Its evidence changed
priorities: the dense-pile fixture is 88.1% Dantzig solve-proper (D3
revisit trigger FIRED; WP-PG.12 deprioritized), and the active-3k
many-islands regime is ~50% integration (WS-C is the lever there).

Claimable now, in priority order:

1. **WP-PG.03** (WS-E — profiling doc and Tracy config; currently claimed).
2. **WP-PG.21** only if a new current-base profile justifies revisiting the
   ODE active path. WP-PG.20 is #3329 and intentionally stayed span-only
   after the map/pruning variant showed small-row overhead; WP-PG.22 and
   WP-PG.11 both have local current-base rejection evidence from 2026-07-06.

Blocked/gated (do not claim): PG.04 (D4), PG.12 (evidence), PG.13
(PG.10 census), PG.14 (D3 — now urgent), PG.15 (D7 — now urgent), PG.23
(D8), PG.33, PG.41. PG.42 is done in PR #3299; WP-PG.30 is done in
PR #3310. **D3 and D7 are the decisions that unblock the dense-pile
fixture**; everything claimable above serves the many-islands and ODE
regimes meanwhile.

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

D1 (SIMD FP contract), D2 (ISA delivery), D3 (matrix-free — **revisit
trigger fired by WP-PG.01**: solve-proper is 88.1% of the dense-pile
step), D4 (executor tooling), D5 (ODE lane depth = PG.20/21/22), D7
(penetration-creep remediation — **S6 confirms 0.362 m penetration,
0/71 resting at baseline**), D8 (manifold reduction now vs WS-F phase 3)
— see README "Open decisions". D3 and D7 are now the highest-leverage
decisions.

## Session log (round-2 execution)

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
  WS-F check found #3281 merged only internal native collision math, so
  this branch remains the DART 6 detector SIMD-consumer path.
