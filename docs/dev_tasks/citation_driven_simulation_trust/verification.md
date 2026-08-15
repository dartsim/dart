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
tests/test_check_citation_evidence.py -q` — 68 passed.
  - `--freshness` run — exit 0 (packet commit == HEAD).
- Negative control: permanent `evidence/negative-controls/` packet (missing
  resolved identity/digest/commands, single-run ensemble, unsupported-as-zero
  metric, no claim boundary) fails with >= 3 errors and the gate enforces
  that it keeps failing.
- Determinism evidence: per-cell trajectory SHA-256 equal across 2 repeats
  for all 14 sweep cells (2 repeats each, 28 runs); cross-invocation reruns reproduced identical summary
  metrics.
- Performance/allocation: explicitly typed unsupported in the packet (no
  timing/allocation claims made).
- Visual evidence: typed not-applicable (numeric rotational-symmetry oracle);
  no visible-behavior claim in this packet.
- Review passes (two independent, role-separated, on commit d0cb640860d):
  a tooling/correctness pass and a physics/evidence-honesty pass. Both
  returned "not clean"; every substantive finding was verified against source
  before acting. Confirmed and fixed in the follow-up slice below:
  1. BLOCKER — `evidence_dir.glob("*.json")` was non-recursive, so a lane
     could close a row with prose, an empty object, or the negative control
     itself one directory deeper. Reproduced in a scratch tree (zero errors
     reported for four bypass variants).
  2. MAJOR — "rejects unsupported-as-zero" was asserted in four places and
     implemented in none, and the shipped CT-001 packet committed exactly
     that: `max_solver_residual: 0.0` is structurally never computed
     (`recordSolverDiagnostics(World&, size_t, double residual = 0.0)` with
     no rigid call site passing a residual, verified at
     `dart/simulation/compute/rigid_body_contact_stage.cpp:165,736,874,926`),
     and BOXED_LCP records no iteration count at all (its branch returns
     before the diagnostics call, verified at the same file, line 898).
  3. MAJOR — `evidence.raw_paths` was never checked for existence.
  4. MAJOR — `target.commit` is the parent commit (the writer stamps HEAD
     before the packet is committed); the measured library state is that
     commit, now stated explicitly in `target.commit_role`.
  5. MINOR — packet did not say which isotropy criterion fired; resolved
     provenance credited stage names that do not discriminate; repeat hashes
     were not recorded; corpus packet skeleton did not match the real schema.
- Known gaps: remaining first-wave families (CT-004 articulated,
  CT-005 control, CT-006 heel-strike, CT-011 reset/concurrency);
  `ResolvedSolverConfiguration` not yet Python-exposed (WS4).
- Changelog: entry added under "Tests, Benchmarks, and Quality Gates".

## Review-fix slice — 2026-08-14

- What changed: validator hardened (recursive plus lane-referenced packet
  validation, negative-control and non-JSON lane references rejected,
  duplicate packet owners rejected, scalar `evidence` guarded, `raw_paths`
  existence, non-empty measurement window, spelled-placeholder rejection,
  empty-container rejection, and the enforced zero rule: every exact zero in
  a measured group must be typed unsupported with a reason or declared in
  `measured_zero_fields`); `scripts/citation_packet_utils.py` centralizes the
  typed-unsupported markers with their source citations; CT-001 regenerated
  with honest metric typing, per-criterion anisotropy findings, a pyramid
  antisymmetry-signature test, a physical-validity gate on the disposition,
  and recorded per-repeat hashes; CT-002 and CT-003 packets added.
- Commands and results:
  - `pixi run check-citation-evidence` — first run after hardening reported
    3 errors, all in this program's own packets (the exact unsupported-as-zero
    defect the reviews named); OK after the packets were corrected.
  - `pixi run python -I scripts/run_pytest.py
tests/test_check_citation_evidence.py -q` — 68 passed (15 new cases
    covering the blocker and each new rule).
  - `PYTHONPATH=build/default/cpp/Release/python pixi run python
scripts/write_citation_ct002_dense_contact_packet.py` — all cells finite;
    max penetration 2.047e-2 m; settled max speed 8.066e-2 m/s; max
    post-settle energy gain 9.594e-10 J; one unstable cell
    (SEQUENTIAL_IMPULSE at dt=4 ms still moving at the horizon) →
    disposition `reproduced`.
  - `PYTHONPATH=... scripts/write_citation_ct003_elastic_contact_packet.py` —
    all cells finite, energy-envelope excess exactly 0.0 J against a
    1.801e-4 J tolerance → disposition `unresolved`: the cited energy
    injection did NOT occur in this bounded reconstruction, which is recorded
    as a negative result rather than a refutation of the original report.
- Negative control: still fails (>= 3 errors) and is now also rejected if a
  lane references it as evidence.
- Known gaps after this slice: CT-002/CT-003 are bounded reconstructions, not
  source-exact SimBenchmark scenes; per-contact cone/complementarity metrics
  remain unavailable until WS4.

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
