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
  - `pixi run check-citation-evidence` — OK.
  - `pixi run python -I scripts/run_pytest.py
    tests/test_check_citation_evidence.py -q` — 33 passed.
- Measured (this commit, boxed-LCP default, sphere slide-to-roll sweep):
  fcl/dart/ode max |lateral drift| 2.101e-3 m and relative travel spread
  4.806e-3; bullet 6.696e-4 m and 9.236e-4; heading errors <= 1.6e-4 deg;
  min final sphere height 0.0800 m (no fall-through); all four detectors
  exceed the isotropy tolerance, so the packet disposition is `reproduced`
  with an explicit claim boundary.
- Negative control: permanent intentionally incomplete packet fails with
  >= 3 errors; the gate rejects a passing negative control as vacuous.
- Performance/allocation: typed unsupported (no timing/allocation claims).
- Visual evidence: typed not-applicable (numeric rotational-symmetry
  oracle).
- ABI/compatibility: additive tooling/docs/tests only; no library, header,
  component, default, or downstream-visible change, so
  `pixi run -e gazebo test-gz` is not triggered by this slice.
- Review passes: recorded below once completed for the slice head.
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
