# Resume: Citation-Driven Simulation Trust

## Current reality

Reconciled against a live checkout on 2026-08-14. The bootstrap-package
statuses were verified; the corpus sidecar now carries the WS0 audit record.

## Last session summary

- WS0 audit done against `main` 20501341226 and `release-6.20` 39ccd52068b:
  plan IDs free (PLAN-123/PLAN-623), PLAN-091 heritage mapped, open PRs
  (#3432 PLAN-104, #3377 FBF friction, #3431 soft-foot ensembles, #3428
  WP-PG.50) and issue #3056 routed as row owners, existing demos scenes
  recorded as fixture seeds.
- PLAN-123 integrated into `docs/plans/dashboard.md` and `docs/plans/README.md`.
- WS1 landed on the DART 7 branch: `claims-manifest.json` (20 rows),
  fail-closed `scripts/check_citation_evidence.py`
  (`pixi run check-citation-evidence`, wired into `check-lint`),
  `tests/test_check_citation_evidence.py`, a permanent intentionally
  incomplete negative-control packet, and the first complete CT-001
  rolling-direction packet (disposition `reproduced`: friction-pyramid
  lateral drift up to 2.1 mm with zero drift at 0/45/90 deg, identical-run
  determinism, per-method requested/resolved readback).

## Current branches

- DART 7: `feature/citation-trust-foundation` in
  `.claude/worktrees/citation-trust-main`, based on `origin/main` 20501341226.
- DART 6: `feature/dart6-citation-contact-trust` in
  `.claude/worktrees/citation-trust-620`, based on `origin/release-6.20`
  39ccd52068b (docs applied; branch-local adoption in progress).

Both are local-only; nothing pushed. GitHub mutations need maintainer
approval.

## Immediate next step

Finish the DART 6 Phase 1 adoption on `release-6.20`: branch-local
`check_citation_evidence.py` + manifest + negative control (no DART 7 API
backports), then run branch gates on both worktrees and record reviews.

After that, Phase 2 continues with the next first-wave family on `main`
(dense inelastic/elastic contact, CT-002/CT-003) reusing
`rigid_restitution_ladder` and a 6x6x6 grid fixture.

## Context that would be lost

- `ResolvedSolverConfiguration` is C++-only (`World::getResolvedConfiguration`);
  packets record resolved identity from World property readback plus step
  profile stage names, and the Python exposure is the first WS4 slice.
- The CT-001 packet must not be quoted as a solver comparison: SI and boxed
  LCP metric summaries coincide to printed precision on the single-contact
  scene while trajectory hashes differ per solver.
- `check-citation-evidence` validates committed packets structurally;
  `--freshness` (packet commit == HEAD) is a packet-writing aid, not CI,
  because squash merges retire topic commits.
- Negative-control packets live under `evidence/negative-controls/` and must
  keep failing validation with >= 3 errors; the gate rejects a passing
  negative control as vacuous.
- dartpy runs from a built tree via
  `PYTHONPATH=build/default/cpp/Release/python pixi run python ...`.

## How to resume

```bash
git worktree list
cd .claude/worktrees/citation-trust-main && git status && git log -3 --oneline
cd ../citation-trust-620 && git status && git log -3 --oneline
pixi run check-citation-evidence   # in citation-trust-main
```

Then continue with the DART 6 adoption or the next first-wave family per the
README status.

## Verification before ending the next session

- Run `pixi run lint` for any repository edit.
- Run `pixi run check-citation-evidence`, `pixi run check-docs-policy`, and
  `pixi run test-ai-infra` for evidence/docs changes.
- Record commands/results in `verification.md` and decisions in
  `decisions.md`.
