# Resume: DART 6 Citation-Driven Contact Trust

## Current reality

Reconciled against a live checkout on 2026-08-14; bootstrap statuses verified.

## Last session summary

- Phase 0 audit done against `release-6.20` 39ccd52068b: PLAN-623 confirmed
  free and added to `docs/plans/dashboard.md`; PLAN-621 owns CT-018 (issue
  #3056 open, PR #3428 active); PLAN-622/PR #3431 own the CT-020
  perturbation-ensemble methodology; PR #3377 (research lane) owns
  exact-Coulomb FBF fixtures; `dart.dynamics.ContactInverseDynamics` already
  exists on this branch (existing-API note for CT-009).
- Phase 1 landed: branch-local
  `docs/design/dart6_citation_driven_contact_trust/claims-manifest.json`
  (20 rows, dart6 lane only, corpus_reference to the `main` corpus),
  fail-closed `scripts/check_citation_evidence.py`
  (`pixi run check-citation-evidence`, wired into `check-lint`),
  `tests/test_check_citation_evidence.py`, and a permanent negative-control
  packet.
- First complete packet: CT-001 rolling-direction detector sweep via
  `scripts/write_citation_ct001_rolling_direction_packet.py` (classic
  pybind11 API, default boxed-LCP solver, detectors swept per build
  availability, deterministic repeats).

## Current branch

`feature/dart6-citation-contact-trust` in
`.claude/worktrees/citation-trust-620`, based on `origin/release-6.20`
39ccd52068b. Local only; nothing pushed. GitHub mutations need maintainer
approval.

## Immediate next step

Run the branch gates (`pixi run lint`, `pixi run check-lint`,
`pixi run test-ai-infra`, `pixi run check-citation-evidence`) and record two
independent review passes; then continue Phase 2 (audit/guard PLAN-621/622
evidence rows) and Phase 3 with the next first-wave contact row not owned
elsewhere (dense inelastic/elastic grids, CT-002/CT-003).

## Context that would be lost

- The `main` PLAN-123 manifest keeps dart6 lanes as routing pointers; THIS
  branch manifest owns `release-6.20` lane status/dispositions. Promotion
  back to the `main` manifest happens explicitly at sync points, never
  automatically.
- Per-solve LCP iterations/residuals and Dantzig-vs-PGS fallback events are
  not exposed on this branch; packets type them unsupported rather than
  fabricating zeros. Adding opt-in diagnostics is future Phase 4 work under
  the compatibility contract.
- dartpy needs `pixi run build-py-dev`
  (`PYTHONPATH=build/default/cpp/Release/python` for scripts); plain
  `pixi run build` does not build dartpy on this branch.
- FreeJoint launch velocities: generalized velocities `[w(3), v(3)]`; the
  writer asserts the `getLinearVelocity()` readback matches the requested
  launch vector.

## How to resume

```bash
git worktree list
cd .claude/worktrees/citation-trust-620 && git status && git log -3 --oneline
pixi run check-citation-evidence
```

Then continue per the README status checkboxes.

## Minimum verification for the next slice

```bash
pixi run lint
pixi run check-lint
git diff --check
```

Add focused tests and `pixi run -e gazebo test-gz` as soon as behavior or a
downstream-sensitive path changes.
