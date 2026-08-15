# PR draft: release-6.20 citation contact trust

Status: OPENED as https://github.com/dartsim/dart/pull/3444 on 2026-08-15
with maintainer approval; this file is the posted body's source. Branch `feature/dart6-citation-contact-trust`, base
`release-6.20`, milestone DART 6.20.0. Delete this file with the dev-task
folder at task completion.

---

Title: `Add the PLAN-623 branch-local citation evidence contract and the
CT-001 detector-sweep packet`

## Summary

- Adopts the citation-trust evidence contract on the LTS branch as additive
  tooling only: a branch-local claims manifest, the fail-closed
  `pixi run check-citation-evidence` gate wired into `check-lint`, a
  permanent negative control, 52 pytest cases, and the first `release-6.20`
  evidence packet — a CT-001 rolling-direction sweep across the fcl, dart,
  bullet, and ode collision detectors.
- No library, API, ABI, default, packaging, or downstream-visible change:
  the diff touches only `docs/`, `scripts/`, `tests/`, `pixi.toml`, and
  `CHANGELOG.md`, verified by independent review.

## Motivation / Problem

- Historical claims about DART 6 contact behavior had no branch-qualified,
  reproducible dispositions on the maintained LTS line. PLAN-623
  (docs/design/dart6_citation_driven_contact_trust.md, dashboard entry
  added here) gives them stable rows and fail-closed packets while
  preserving the compatibility contract; claim identity stays owned by the
  `main` corpus via `corpus_reference`, so rows cannot fork across branches.

## Changes / Key Changes

- Branch-local `claims-manifest.json` (single `dart6` lane per row) under
  the design-doc sidecar, so evidence survives dev-task cleanup.
- `scripts/check_citation_evidence.py` — the same fail-closed packet
  contract as `main`, adapted to `release-6.20` lane ownership; in
  `check-lint`; permanent negative control; 52 tests in `test-ai-infra`
  (the pinned task list in `scripts/check_ai_infrastructure.py` is extended
  in the same change, which is how that guard is designed to grow).
- CT-001 packet: one sphere launched sliding across a swept in-plane angle,
  default boxed-LCP solver, per-run detector readback asserted, two
  bit-identical repeats per cell. The claim reproduces on fcl, dart, and ode
  (lateral drift 2.1e-3 m with the antisymmetric friction-pyramid
  signature: nulls at 0/45/90 deg, antisymmetry residual ~1e-15 of peak);
  bullet exceeds the drift tolerance without that angular structure
  (residual 1.79x its own peak, largest drift near 45 deg where the
  mechanism predicts zero) and is excluded from the reproducing set rather
  than counted as a fourth corroboration. dart and ode are bit-identical at
  every angle and the packet says so.
- Per-solve LCP iterations, residuals, and Dantzig-vs-PGS fallback events do
  not exist as public API on this branch; they are typed
  `{status: unsupported, reason}` rather than reported as zeros.

## Testing

- `pixi run check-lint` (includes the new gate) and `pixi run test-all` —
  158/158 tests pass on the branch head.
- `pixi run python -I scripts/run_pytest.py
  tests/test_check_citation_evidence.py -q` — 52 passed.
- `pixi run -e gazebo test-gz` not triggered: the diff has no collision,
  constraint, `World::step`, parser, package, or downstream-sensitive
  change (documented in the dev-task verification log with the file list).
- Three rounds of independent role-separated review; all findings fixed
  in-branch and recorded, including two corrections to this branch's own
  records (an imported wrong-branch verification block and a wrong
  largest-drift angle).

## Breaking Changes

- [x] None (additive docs/scripts/tests/pixi only; LTS contract preserved).

## Related Issues / PRs (backports)

- Companion `main` PR: branch `feature/citation-trust-foundation`
  (independent; owns the corpus, PLAN-123, and the DART 7 packets).

#### Checklist

- [ ] Milestone DART 6.20.0
- [x] CHANGELOG.md updated (DART 6.20.0 "Tests")
- [x] Unit tests added (validator suite)
- [x] No new public methods/classes (docs in script docstrings)
- [x] Python bindings not applicable (no API change)
