# Verification: DART 6 Citation-Driven Contact Trust

## Phase 0 + Phase 1 slice — 2026-08-14

- Branch: `feature/dart6-citation-contact-trust` from `origin/release-6.20`
  39ccd52068b in `.claude/worktrees/citation-trust-620`; worktree started
  clean.
- What changed: PLAN-623 dashboard entry; branch-local
  `docs/design/dart6_citation_driven_contact_trust/claims-manifest.json`;
  fail-closed `scripts/check_citation_evidence.py` (task
  `check-citation-evidence`, wired into `check-lint`);
  `tests/test_check_citation_evidence.py` (registered in `test-ai-infra`);
  permanent negative control under `evidence/negative-controls/`;
  `scripts/write_citation_ct001_rolling_direction_packet.py` and the CT-001
  packet; CHANGELOG entry under DART 6.20.0 Tests.
- Commands and results:
  - `pixi run build` — success; `pixi run build-py-dev` — success (dartpy
    with fcl, dart, bullet, and ode detectors bound).
  - `PYTHONPATH=build/default/cpp/Release/python/dartpy pixi run python
    scripts/write_citation_ct001_rolling_direction_packet.py` — wrote the
    packet; per-cell deterministic repeats identical (56 runs);
    requested-vs-readback detector identity asserted per run.
  - `pixi run lint` — clean; `pixi run check-lint` (full aggregate) — exit 0.
  - `pixi run check-citation-evidence` — OK.
  - `pixi run python -I scripts/run_pytest.py
    tests/test_check_citation_evidence.py -q` — 48 passed.
- Measured (this commit, boxed-LCP default, sphere slide-to-roll sweep):
  fcl/dart/ode max |lateral drift| 2.101e-3 m and relative travel spread
  4.806e-3; bullet 6.696e-4 m and 9.236e-4; heading errors <= 1.6e-4 deg;
  min final sphere height 0.0800 m (no fall-through). Only fcl, dart, and ode
  carry the antisymmetric friction-pyramid signature; bullet exceeds the
  drift tolerance without angular structure (largest drift at 45 deg, where
  the mechanism predicts zero) and is explicitly excluded from the
  reproducing set. dart and ode are bit-identical, so the sweep holds three
  independent behaviors, not four. Disposition `reproduced` for fcl/dart/ode
  under a validity gate (rolling speed 5/7 v0, no fall-through, bounded
  penetration) plus an explicit claim boundary.
- Negative control: permanent intentionally incomplete packet fails with
  >= 3 errors; the gate rejects a passing negative control as vacuous.
- Performance/allocation: typed unsupported (no timing/allocation claims).
- Visual evidence: typed not-applicable (numeric rotational-symmetry
  oracle).
- ABI/compatibility: additive tooling/docs/tests only; no library, header,
  component, default, or downstream-visible change, so
  `pixi run -e gazebo test-gz` is not triggered by this slice.
- Review passes (two independent, role-separated, on commit 1189155fe62):
  a tooling/LTS-compatibility pass and a physics/evidence-honesty pass. Both
  returned "not clean"; findings verified against source before acting:
  1. MAJOR — the validator's non-recursive `evidence/*.json` enumeration let
     a lane close a row with a file the packet checks never reached
     (reproduced: a lane pointing at `evidence/sub/p.json` containing
     `{"schema": "WRONG"}` produced zero errors). Fixed by validating every
     lane-referenced path wherever it sits and rejecting non-JSON, outside,
     and negative-control references.
  2. MAJOR — bullet was listed as symmetry-breaking on scatter with no
     angular structure. Measured antisymmetry residual relative to each
     detector's own peak drift: fcl 4.607e-15 / 2.101e-3 = 0.000, dart and
     ode 1.554e-15 / 2.101e-3 = 0.000, bullet 1.201e-3 / 6.697e-4 = 1.793 —
     bullet's asymmetry exceeds its own signal, its largest drift sits at
     45 deg where the mechanism predicts zero, and its penetration/energy
     gain are ~1000x/~6000x the others. The packet now applies that
     antisymmetry-signature test (threshold 5% of peak) and excludes bullet
     from the reproducing set with an explicit limitation.
  3. MAJOR — the disposition had no physical-validity gate, so a degenerate
     run would have strengthened "reproduced". A validity gate (rolling speed
     5/7 v0, rolling actually reached, no fall-through, bounded penetration)
     now forces `unresolved` if any cell degenerates.
  4. MINOR — dart and ode are bit-identical at every angle; recorded in
     `metrics.physical.identical_detector_groups` and in the limitations, so
     the sweep is not read as four independent measurements.
  5. MINOR — `raw_paths` existence unchecked, string metric leaves accepted,
     unsupported LCP internals carried as free prose rather than typed
     markers. All fixed; per-solve iterations, residual, and fallback events
     are now typed `unsupported` with branch-specific reasons.
  Compatibility was reviewed clean: the diff touches only `docs/`, `scripts/`,
  `tests/`, `pixi.toml`, and `CHANGELOG.md`, with nothing under `dart/`,
  `python/dartpy/`, `cmake/`, or packaging.
- Round-2 verification (post-fix, independent): confirmed the bypass is
  closed, the antisymmetry attribution matches recomputation from raw_rows,
  and the source claims about `recordSolverDiagnostics` and the BoxedLcp
  branch are exact in the current tree. It also refined one claim: an opt-in
  AVBD stage can record an iteration count earlier in the same step, so
  "never recorded for this method" was over-broad. The marker wording and the
  helper now key off the observed value rather than the method name, so a
  genuinely recorded count is published instead of being laundered as
  unsupported. Resolved-configuration assertions were widened to cover
  detector, timestep, and gravity per run rather than detector alone.
- Known gaps: Phase 2 (guard PLAN-621/622 evidence) and remaining first-wave
  rows; per-solve LCP diagnostics remain typed unsupported.

## Bootstrap record — 2026-08-14

### What changed

Documentation package only:

- DART 6 compatibility/evidence design;
- branch-local active task contract;
- dashboard snippet and cross-branch goal prompt.

### What is not claimed

- No live DART 6 checkout was edited.
- No build, test, benchmark, visual, ABI, package, or Gazebo gate ran.
- No branch/commit/PR/issue/GitHub state changed.
- Corpus dispositions and plan ID require current audit.

### Required integration checks

```bash
pixi run lint
pixi run check-lint
git diff --check
```

Use additional docs policy/spelling tasks present on the current branch.

### Required behavioral record

For every later slice record:

- exact release base and candidate head;
- corpus row and branch-qualified claim;
- negative-control baseline;
- detector/solver/timestep/threads/seed/window;
- state/contact/rest hashes and explicit re-baselines;
- raw metric and timing packet;
- allocation and determinism result;
- ABI/header/component/package assessment;
- Gazebo/gz command and result when affected;
- visual evidence and semantic review when applicable;
- independent reviews, limitations, changelog decision, and next step.
