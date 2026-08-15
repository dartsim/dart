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
- First-wave packets landed: CT-001 (both branches), CT-002, CT-003, and
  CT-004 (`main`). Four of the six capped families now have at least one
  branch-qualified packet. Dispositions: CT-001 and CT-002 `reproduced`,
  CT-003 and CT-004 `unresolved` -- in both cases because the cited
  behavior was not observed, which the packets state plainly rather than
  reaching for a positive verdict.

## Current branches

- DART 7: `feature/citation-trust-foundation` in
  `.claude/worktrees/citation-trust-main`, based on `origin/main`
  20501341226, at `bfa4f0c99bd` (15 commits): the contract and gate, packets for
  all six first-wave families (CT-001..005, 007, 011), the
  resolved-configuration bindings, three review-fix passes, and the CT-007 /
  CT-011 issue drafts plus both PR drafts.
- DART 6: `feature/dart6-citation-contact-trust` in
  `.claude/worktrees/citation-trust-620`, based on `origin/release-6.20`
  39ccd52068b, at `978b48e6653` (7 commits).

Both local-only; every head has `check-lint` + `test-all` + the citation
gate green. GitHub mutations need maintainer approval.

## Immediate next step

Awaiting maintainer decisions: post the CT-007 and CT-011 issues
(`ct007-issue-draft.md`, `ct011-issue-draft.md`), and push/open the two PRs
(`pr-draft-main.md` here, `pr-draft-620.md` on the 6.20 branch). After that,
the next implementation slice is WS3 contact ordering/identity on a NEW
branch (this one is bounded per the work-package rules), motivated concretely
by CT-011's hidden contact state, followed by CT-006 heel-strike and the
per-solve residual (WS4 remainder).

## Context that would be lost

- The gate is fail-closed in ways worth knowing before writing a packet:
  every exact zero in a measured metric group must be typed
  `{"status": "unsupported", "reason": ...}` or listed in
  `measured_zero_fields`, and a stale declaration (a listed field that is not
  zero) also fails. Use `scripts/citation_packet_utils.py` for the shared
  markers rather than inventing wording.
- Quantities that do not exist on `main` today, and must stay typed
  unsupported until WS4 lands them: per-solve solver residual (rigid contact
  never passes one to `recordSolverDiagnostics`), boxed-LCP iteration counts
  (its branch returns before the diagnostics call), per-island fallback
  reporting, and `ResolvedSolverConfiguration` in Python.
- Packet dispositions must survive an adversarial read. Review killed two
  verdicts: one rested on a hardcoded threshold (a settle speed exactly
  linear in dt is integrator residual, not instability), and one measured
  something the cited claim never asserted (CT-004 convergence against a
  claim about matched cost). Check the disposition against the corpus
  definition -- "the claimed behavior is observed" -- not against whether
  the fixture produced an interesting result.
- `target.commit` is the commit the library was measured at; the packet and
  its writer necessarily land one commit later. `target.commit_role` says so.
- dartpy runs from a built tree:
  `PYTHONPATH=build/default/cpp/Release/python pixi run python ...` on
  `main`, and `.../python/dartpy` on `release-6.20` (which also needs
  `pixi run build-py-dev`).
- The `main` manifest keeps dart6 lanes as routing pointers only; the
  `release-6.20` branch manifest owns that branch's lane state.

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
