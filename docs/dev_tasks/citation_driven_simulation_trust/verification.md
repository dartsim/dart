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

## Round-2 verification and second fix pass — 2026-08-14

Two independent verifiers re-checked the post-fix state. Both confirmed the
bypass is closed and the zero rule is real, and both found further defects in
the CT-002/CT-003 packets that were then fixed:

1. CT-002's `reproduced` verdict rested on one cell crossing a hardcoded
   0.05 m/s settle threshold. The verifier showed the settle speed is
   bit-exactly linear in dt (`speed/dt` identical to 17 digits across the two
   timesteps), which is a converging integrator's quasi-static residual, not
   instability. The packet now computes that dt-linearity explicitly and
   refuses to count a dt-linear excess as instability.
2. Replacing it, CT-002 gained a real oracle: total mechanical energy must not
   increase after the pile settles. Measured per-step gain in the settle
   window is 9.98e-4 J (2 ms) and 1.31e-3 J (4 ms) under SEQUENTIAL_IMPULSE
   against a 1.80e-4 J tolerance, while BOXED_LCP shows 0.0 and 1.34e-6 J.
   That solver-specific, non-divergent energy gain is what the packet now
   reports as reproducing the claim, with the scaling limb marked
   uninstrumented. The first attempt at this oracle measured the whole run and
   fired on all four cells; that was free-fall discretization error, so the
   window is gated to the settled phase.
3. `configuration.resolved_provenance` claimed per-cell assertion of four
   fields while only the contact solver was checked, and `setdefault` kept the
   first timestep so 4 ms cells were published as 2 ms. Resolved identity is
   now keyed per cell (`METHOD@dt=...`) with contact solver, timestep, and
   gravity each asserted; the same widening was applied to CT-001 and to the
   `release-6.20` writer.
4. `solver_iterations_by_method()` keyed "unsupported" off the method name, so
   a genuinely recorded AVBD-sourced count would have been laundered into
   "unsupported". It now keys off the observed value, and the BOXED_LCP marker
   no longer claims the count is _never_ written.
5. CT-002 cited an "energy monotonicity" oracle it did not have (it tracked
   kinetic energy only); CT-003's envelope maximum was set by its seed value.
   Both are now transparent: CT-002 tracks total energy, and CT-003 records
   `max_total_energy_after_step_j` and `envelope_set_by_initial_state` so the
   one-sidedness of the test is visible.

- Commands after the second fix pass: `pixi run check-citation-evidence` — OK
  (it caught two further unacknowledged zeros during the rework, both now
  declared as genuine measurements); 68 pytest cases pass.
- Known gaps after this slice: CT-002/CT-003 are bounded reconstructions, not
  source-exact SimBenchmark scenes; per-contact cone/complementarity metrics
  remain unavailable until WS4; the solver-failure limb of CT-003 is only
  instrumented as non-finite state.

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
