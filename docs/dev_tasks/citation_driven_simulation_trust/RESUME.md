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
- WS1 landed on both branches: checked claims manifests, fail-closed
  `check-citation-evidence` gates wired into `check-lint`, permanent
  intentionally incomplete negative controls, and pytest suites (68 cases on
  `main`, 48 on `release-6.20`).
- Two independent role-separated reviews ran per branch. All four returned
  "not clean". Every finding was verified against source and fixed; the
  substantive ones were a validator bypass (a lane could close a row with a
  file the packet checks never reached), unsupported-as-zero metrics in this
  program's own packets, and a DART 6 attribution error crediting bullet with
  a friction-pyramid signature its data does not show. See `verification.md`.
- First-wave packets landed: CT-001 (both branches), CT-002 and CT-003
  (`main`).

## Current branches

- DART 7: `feature/citation-trust-foundation` in
  `.claude/worktrees/citation-trust-main`, based on `origin/main` 20501341226.
- DART 6: `feature/dart6-citation-contact-trust` in
  `.claude/worktrees/citation-trust-620`, based on `origin/release-6.20`
  39ccd52068b (docs applied; branch-local adoption in progress).

Both are local-only; nothing pushed. GitHub mutations need maintainer
approval.

## Immediate next step

Continue Phase 2 on `main` with the articulated energy/momentum family
(CT-004): a passive multi-link chain swept over timestep, using
`StepMetrics.total_energy`/`angular_momentum` as the oracle and the PLAN-084
variational integrator as the comparison arm, written with
`scripts/citation_packet_utils.py` so unsupported quantities stay typed.

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
