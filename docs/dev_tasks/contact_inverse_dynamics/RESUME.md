# Resume: Contact-Aware Inverse Dynamics + IK Utilities (DART 6.19)

## Last Session Summary

Core feature is implemented and committed on
`feature/contact-inverse-dynamics-6.19`: NNLS solver in `dart/math`,
`ContactInverseDynamics` in `dart/dynamics`, unit tests (22 passing),
benchmarks plus a measured NNLS dual-tolerance optimization (see
`02-perf.md`). dartpy bindings + Python tests written (build verifying), GUI
example `examples/contact_inverse_dynamics` in progress.

## Current Branch

`feature/contact-inverse-dynamics-6.19` — commits:
dev-task notes, NNLS, ContactInverseDynamics, benchmarks+perf.

## Immediate Next Step

Verify dartpy build/tests (`pixi run test-py`) and the GUI example build +
headless smoke (`--frames 30 --screenshot`), then: CHANGELOG 6.19.0 entry
(needs PR number), `pixi run lint` + `pixi run test-all`, adversarial review
pass, delete this dev-task folder in the final PR, push same-named branch
(jslee02 account) and open PR against `release-6.19`.

## Context That Would Be Lost

- No QP solver was needed: friction-pyramid generators + Lawson–Hanson NNLS
  (`01-design.md` records the gate decision and rejected alternatives).
- Perf evidence and before/after table: `02-perf.md`, raw `benchmarks.json`.
- `Joint::getBodyConstraintWrench()` was already bound in dartpy; no new C++
  API was needed for the per-axis torque/structural wrench question.
- Local and remote branch names must match on push; never push to
  `release-6.19` directly.

## How to Resume

```bash
git checkout feature/contact-inverse-dynamics-6.19
git status && git log -5 --oneline
./build/default/cpp/Release/tests/unit/dynamics/UNIT_dynamics_ContactInverseDynamics
```

Then: continue from "Immediate Next Step".
