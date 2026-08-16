# PR draft: DART 7 citation-trust foundation

Status: OPENED as https://github.com/dartsim/dart/pull/3445 on 2026-08-15
with maintainer approval; this file is the posted body's source. Branch `feature/citation-trust-foundation`, base `main`, milestone
DART 7.0. Delete this file with the dev-task folder at task completion.

---

Title: `Add the PLAN-123 citation-trust foundation: fail-closed evidence
contract, six first-wave packets, and resolved-configuration bindings`

## Summary

- Lands PLAN-123's foundation: a machine-checked citation-claim manifest and
  a fail-closed `pixi run check-citation-evidence` gate (wired into
  `check-lint`), first evidence packets for all six capped first-wave
  fixture families, and Python bindings for the World's bake-time
  `ResolvedSolverConfiguration` so packets record the method that actually
  ran.
- Two findings surfaced by the packets are filed as #3442 and #3443: the default `SEQUENTIAL_IMPULSE` contact
  solver lets a heavy box sink fully through a light one at mass ratios of
  100 and above (CT-007), and `World.state_vector` restore is not a function
  of the restored state once the world has contact history (CT-011).

## Motivation / Problem

- External claims about DART (SimBenchmark, exoskeleton contact-force
  criticism, exact-cone papers, Nimble/RobotDART workflow needs) had no
  branch-qualified, reproducible dispositions: no owner converted them into
  stable rows with source-bound evidence, and benchmarks could name a
  requested method without proving it ran.
- PLAN-123 (docs/plans/123-citation-driven-simulation-trust.md, landed here
  with its durable design doc docs/design/contact_trust_and_observability.md)
  makes every such claim a stable row with a fail-closed evidence packet.

## Changes / Key Changes

- Claim/evidence contract: `claims-manifest.json` (20 corpus rows, exact-ID
  agreement with the human corpus enforced), packet schema
  `dart.citation_claim_evidence/v1`, validator
  `scripts/check_citation_evidence.py` in `check-lint`, permanent
  intentionally incomplete negative control that must keep failing, 85
  pytest cases. The gate rejects prose/non-JSON/outside-`evidence/`/
  negative-control/non-string lane references, dangling or directory
  `raw_paths`, scene digests that disagree with the published parameters,
  single-run ensembles, spelled placeholders, and any exact zero not
  declared a measurement or typed `unsupported` with a reason.
- First-wave packets (writers under `scripts/`, packets under the plan's
  `evidence/`): CT-001 rolling-direction (`reproduced`: friction-pyramid
  antisymmetry signature), CT-002 dense inelastic (`reproduced`: settled-pile
  energy gain under sequential impulse only), CT-003 dense elastic
  (`unresolved`: no energy injection observed), CT-004 articulated energy
  drift (`unresolved`: converges, but the cited claim is matched-cost),
  CT-005 PD tracking (`unresolved`: controller-limited error), CT-007 high
  mass-ratio (`unresolved` for the comparative claim; baseline finding
  above), CT-011 restore equivalence (`unresolved`; finding above).
- WS4 slice 1: nanobind bindings for `ResolvedConfigurationNote` /
  `ResolvedSolverConfiguration`, the `World.resolved_configuration`
  property, surgical stub entries, and two Python tests; every packet
  records per-domain requested/resolved/reason/substitution from the World
  itself.
- PLAN-123 registered in the dashboard/plan index; corpus carries the WS0
  audit; dev task holds the working state, verification log (three
  independent review rounds and their fixes), and the two issue drafts.

## Testing

- `pixi run check-lint` (includes the new `check-citation-evidence` gate) and
  `pixi run test-all` — all phases pass on the branch head.
- `pixi run python -I scripts/run_pytest.py
tests/test_check_citation_evidence.py -q` — 85 passed.
- Every packet regenerated on the current writers; deterministic repeats
  bit-identical; dispositions computed from recorded data with negative
  controls and physical-validity gates.
- Three rounds of independent role-separated review; all findings fixed
  in-branch and recorded in the dev-task verification log.

## Breaking Changes

- [x] None (additive docs/tooling/tests plus additive Python bindings).

## Related Issues / PRs (backports)

- Companion release-6.20 PR: branch `feature/dart6-citation-contact-trust`
  (independent; adopts the same contract without DART 7 APIs).
- Files #3442 (CT-007 default-solver high-mass-ratio failure) and #3443
  (CT-011 restore-history dependence); both packets and issue sources are in
  this PR.

#### Checklist

- [ ] Milestone DART 7.0
- [x] CHANGELOG.md updated ("Tests, Benchmarks, and Quality Gates")
- [x] Unit tests added (validator suite; World resolved-configuration tests)
- [x] New bindings documented via docstrings and stubs
- [x] Python bindings added (`World.resolved_configuration`)
