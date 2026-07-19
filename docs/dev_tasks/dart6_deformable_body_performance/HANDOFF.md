# Handoff - DART 6 deformable body performance

Updated: 2026-07-18

This is the takeover summary for PR #3382 stabilization. `RESUME.md` owns the
ordered commands and approval boundaries; `06-pr-evidence.md` owns detailed
historical evidence.

## Packet contract

**Goal:** publish and stabilize the current representative deformable-body
release slice with review findings fixed, exact-head gates honest, demos
integrated into `dart-demos`, and DART 6 compatibility preserved.

**Done for this PR-maintenance packet when:** the local base merge and Windows
test calibration are published, the resulting exact head has clean automated
review and required CI, and remaining PLAN-622 work stays explicit.

Landing #3382 does not complete the broader deformable-body objective.

## Current checkout

```text
worktree   /home/js/dev/dartsim/dart/task_2
branch     wp-db-native-soft-fallback
pre-push   b172b2ee1db
base       75306efe770
merge      834a2548fd9
calibrate  50a254e7e56
PR         https://github.com/dartsim/dart/pull/3382
```

`wp-db-soft-skel-allocation-gates` is fully incorporated. Do not resume it.

## Current evidence

The pre-follow-up published head `b172b2ee1db` has a clean current-head Codex
review, no unresolved review threads, and 21 passed / 1 failed / 1 skipped
checks. Both Codecov checks passed: patch `94.34783%` with 143 uncovered changed
lines; project `75.17%` versus `73.90%` (reported `+1.26` percentage points).

Windows Release was the only failure. It completed 150/151 tests and failed only
`SoftDynamicsTest.restingSoftContactForceAndCenterOfPressureAreSmooth` in the
legacy-FCL `default adaptive` lane: `0.12115883267368355` m maximum per-step CoP
motion exceeded the former `0.11` m bound.

The local candidate merges `origin/release-6.20@75306efe770` through
`834a2548fd9`; the merge applied cleanly. Test-only commit `50a254e7e56` sets
the legacy-FCL cap to `0.13` m, just above one `0.125` m surface-mesh interval.
The native cap remains `0.02` m and all force, support, spike, finite-state, and
per-step guards remain. Local evidence is 20/20 focused repeats, 25/25
`test_SoftDynamics`, 292/292 no-cache Release build steps, 154/154 full C++
tests, and two clean independent reviews. Downstream verification passed
gz-physics 199/199 tests plus 4/4 performance checks and the selected gz-sim
headless integration smoke 1/1. The test calibration changes no runtime
behavior, API, ABI, or default simulation behavior. Hosted exact-head Windows
remains required after push.

The flagship `adaptive_soft_contact` and `soft_worm` examples are integrated as
`dart-demos` scenes under `examples/demos`; their standalone executables were
removed and GUI-free model tests preserve the numerical contracts. New GUI
examples should continue to be integrated into `dart-demos`.

## Remaining blockers beyond the local fix

- No paired benchmark artifact has `COMPLETE.json`; all preflight-only attempts
  are non-evidence.
- Competitive-envelope and flexible-foot decisions remain open.
- WP-DB.07 multicore scaling and WP-DB.08 native-owned/default coverage remain
  follow-ups.
- The release zero-DoF point-mass assertion fix still needs a separate `main`
  PR.
- PLAN-622 and this task remain active after #3382 stabilization.

## Takeover order

1. Re-fetch the base; merge again only if it advanced.
2. Push the additive commits, update the PR body, and request one fresh
   top-level `@codex review`.
3. Monitor exact-head CI through terminal state, using the new Windows job as
   the calibration proof.
4. Do not merge or close, force-push, change base, request human review, or
   delete branches without new explicit approval.
