# Verification - DART 6 deformable body performance

Updated: 2026-07-19

## Current publication candidate

- Base: `origin/release-6.20@75306efe770`
- Base merge: `834a2548fd9`
- Published PR head: `05d9de6e3fb`
- Test calibration commit: `50a254e7e56`
- DART detector ABI fix: `9a6796596bc`

## Local gates

- Legacy-FCL CoP calibration: 20/20 focused repeats passed.
- `test_SoftDynamics`: 25/25 passed.
- No-cache Release build: 292/292 passed.
- Full Release C++ suite: 154/154 passed.
- Downstream gz-physics suite: 199/199 tests passed.
- Downstream gz-physics performance checks: 4/4 passed.
- Selected gz-sim headless `INTEGRATION_entity_system` smoke: 1/1 passed.
- Independent post-fix reviews: clean x2.
- `pixi run lint`: passed.
- `git diff --check`: passed.
- `pixi run check-ai-commands`: passed after the task-doc refresh.
- ABI follow-up no-cache focused dependency build: passed.
- `test_DARTCollisionDetector`: 22/22 passed.
- `test_Collision`: 52/52 passed.
- `test_NonFiniteContact`: 4/4 passed.
- Exact-v6.19.4-header/new-library ABI canary: 20/20 process runs passed;
  legacy/current detector sizes were both 32 bytes, and current-header thread
  and soft-option configuration, cloning, and active-pool destruction preserved
  the legacy derived canaries and surrounding guards.
- Independent post-ABI-fix reviews: clean x2.

## Hosted evidence

At `b172b2ee1db`, Windows Release completed 150/151 tests; only the legacy-FCL
`default adaptive` CoP assertion failed (`0.12115883267368355` m versus `0.11`
m). Codecov passed with patch `94.34783%` (143 uncovered changed lines) and
project `75.17%` versus `73.90%` (reported `+1.26` percentage points).

Published head `05d9de6e3fb` had successful API docs, Windows Release,
gz-physics, AVX, and both Read the Docs checks at the last refresh. Additional
matrix jobs were running or queued with no CI failure reported. Its fresh top-level
Codex review contains the detector-layout finding addressed by local commit
`9a6796596bc`; there is no inline review thread to resolve.

## Oracle disposition

No visual rerun is required for the 0.11-to-0.13 change because it modifies a
numeric test threshold, not simulation or rendering behavior; visual evidence
cannot prove a per-step CoP limit. The Windows trace, 0.125 m scene mesh
interval, focused repetitions, full soft-dynamics test, full C++ suite, and
downstream gz-physics/gz-sim gates are the relevant text oracle. Historical
visual commands and verdicts remain recorded in `06-pr-evidence.md`, but their
temporary captures no longer exist and are not durable current evidence; the
model gates remain the numerical scene evidence.

## Open evidence

No paired benchmark directory contains `COMPLETE.json`; interrupted directories
are non-evidence. Exact-head Windows CI and a fresh clean automated review
remain required after the next push. Broader PLAN-622 acceptance remains open.
