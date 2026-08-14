# Verification: Citation-Driven Simulation Trust

## WS0 + WS1 slice — 2026-08-14

- Branch: `feature/citation-trust-foundation` from `origin/main` 20501341226
  in `.claude/worktrees/citation-trust-main`; worktree started clean.
- Corpus rows touched: audit section added for all rows; CT-001 `dart7` lane
  `in-progress` with the first packet; CT-002 used by the permanent
  negative control.
- What changed: PLAN-123 docs integrated (dashboard/README/plan/corpus);
  `claims-manifest.json`; `scripts/check_citation_evidence.py`;
  `scripts/write_citation_ct001_rolling_direction_packet.py`;
  `tests/test_check_citation_evidence.py`; pixi tasks
  (`check-citation-evidence` in `check-lint`, test registered in
  `test-ai-infra`); CT-001 packet + negative control; CHANGELOG entry.
- Commands and results:
  - `pixi run build` — success (Release, dartpy 7.0.0 importable).
  - `PYTHONPATH=build/default/cpp/Release/python pixi run python
scripts/write_citation_ct001_rolling_direction_packet.py` — wrote the
    packet; deterministic repeats identical; per-method resolved readback
    asserted; disposition `reproduced`.
  - `pixi run check-citation-evidence` — OK (exit 0).
  - Fail-closed demo: copying the negative control into `evidence/` in a
    scratch tree produced 15 errors and exit 1.
  - `pixi run python -I scripts/run_pytest.py
tests/test_check_citation_evidence.py -q` — 53 passed.
  - `--freshness` run — exit 0 (packet commit == HEAD).
- Negative control: permanent `evidence/negative-controls/` packet (missing
  resolved identity/digest/commands, single-run ensemble, unsupported-as-zero
  metric, no claim boundary) fails with >= 3 errors and the gate enforces
  that it keeps failing.
- Determinism evidence: per-cell trajectory SHA-256 equal across 2 repeats
  for all 28 cells; cross-invocation reruns reproduced identical summary
  metrics.
- Performance/allocation: explicitly typed unsupported in the packet (no
  timing/allocation claims made).
- Visual evidence: typed not-applicable (numeric rotational-symmetry oracle);
  no visible-behavior claim in this packet.
- Review passes: recorded below once completed for the slice head.
- Known gaps: DART 6 adoption slice pending; remaining first-wave families
  pending; `ResolvedSolverConfiguration` not yet Python-exposed.
- Changelog: entry added under "Tests, Benchmarks, and Quality Gates".

## Bootstrap record — 2026-08-14

### What changed

Documentation package only:

- durable DART 7 contact-trust design;
- PLAN-123 and citation corpus;
- DART 7 and DART 6 dev-task contracts;
- merge-safe dashboard/index snippets;
- Codex/Claude goal prompt.

### What is not claimed

- No repository checkout was modified.
- No branch, commit, PR, issue, or GitHub state was created or changed.
- No current-head build, test, benchmark, visual, or solver result was run.
- Corpus dispositions remain audit-required.
- Plan IDs and dashboard ordering require current checkout verification.

### Required verification after integration

At minimum:

```bash
pixi run lint
pixi run check-lint-md
pixi run check-lint-spell
pixi run check-docs-policy
git diff --check
```

Use current branch-owned task names if they differ.

### Required record for each implementation slice

Add a dated section with:

- branch, base, head, and whether the worktree was clean;
- corpus rows and claim boundaries changed;
- source/model/evidence digests;
- code/docs/tests changed at a high level;
- exact commands and results;
- negative control;
- determinism/ensemble evidence;
- performance host validity and raw packet path when applicable;
- visual capture and semantic review when applicable;
- independent review passes;
- known gaps and immediate next step;
- changelog decision.

Never replace raw evidence with a summary number.
