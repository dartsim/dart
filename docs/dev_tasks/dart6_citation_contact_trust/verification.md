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
  drift tolerance without angular structure (its 45 deg drift is 90% of its
  largest, where
  the mechanism predicts zero) and is explicitly excluded from the
  reproducing set. dart and ode are bit-identical, so the sweep holds three
  distinct trajectory sets, not four. (fcl still agrees with dart/ode to ~6e-7 relative, and the friction pyramid under test lives in the shared BoxedLcpConstraintSolver, so the detectors are not independent implementations of the mechanism.) Disposition `reproduced` for fcl/dart/ode
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
     45 deg, where the mechanism predicts zero, is 90% of its largest, and
     its penetration/energy
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
- Round-2 verification (post-fix, independent, on this branch): confirmed the
  lane-reference bypass is closed (nested non-packet, negative control,
  non-JSON, outside-evidence, and shared-owner references all rejected, with
  the real tree validating at zero errors as a control); confirmed every
  antisymmetry number in the packet reproduces exactly from its own
  `raw_rows`; confirmed the disposition validity gate is real by driving
  `build_packet()` against a stubbed dartpy (each of the four conditions
  independently forces `unresolved`); confirmed the typed-unsupported reasons
  are factually true for `release-6.20` (no `getNumIterations`- or
  `getResidual`-style accessor exists anywhere in `dart/constraint/`;
  `BoxedLcpSolver::solve()` returns a bare `bool` and
  `PgsBoxedLcpSolver::Option::mMaxIteration` is an input budget, not an
  achieved count); and confirmed the LTS compatibility gate (16 changed
  files, none under `dart/`, `python/`, `cmake/`, `CMakeLists.txt`,
  packaging, `.github/`, or `pixi.lock`).
- Correction (2026-08-14): an earlier draft of this block described
  `recordSolverDiagnostics`, an AVBD fall-through, and a
  `solver_iterations_by_method` helper. Those belong to the DART 7 `main`
  lane; none of that code exists on `release-6.20`, where
  `scripts/citation_packet_utils.py` is four static markers with no
  conditional logic. The text was imported in error and has been replaced by
  the branch-local record above. AVBD is explicitly out of scope here (see
  `decisions.md` and the branch design doc).
- Correction (2026-08-14): bullet's largest lateral drift is at 75 deg
  (6.696e-4 m), not 45 deg. Its 45 deg drift is 6.004e-4 m -- 90% of its
  largest, where the mechanism predicts exactly zero -- which is still a
  failure of the pyramid signature, and the exclusion rests on the 1.79
  antisymmetry ratio regardless. The packet limitation and this record now
  state it correctly.
- Correction (2026-08-14): the commit message of `afc6d7ac3c2` claimed two
  changes it did not make on this branch -- "scope the iteration marker" and
  deriving `deterministic_repeats_identical` from the comparison. Both were
  DART 7 changes; on `release-6.20` the marker file is static and the field
  was already derived by the preceding commit. Recorded here rather than
  rewritten, since the branch is unpushed but the log should not be silently
  edited to hide the overclaim.
- Note: `deterministic_repeats_identical` is structurally always `true`,
  because a hash mismatch raises `SystemExit` before the packet is built. It
  is a fail-closed guarantee, not a measurement; the independently checkable
  evidence is `raw_rows[*].repeat_trajectory_sha256`.
- Follow-up applied: the disposition validity gate now also fails a run whose
  per-step kinetic-energy gain exceeds a thousandth of the launch energy
  (5.0e-4 J here). Measured: bullet 1.925e-4 J, fcl/dart/ode ~3.0e-8 J, so no
  cell trips it, but a blow-up can no longer pass the gate while a symmetry
  verdict is drawn.
- Final review (independent, on `d4f95f1298f`): compatibility gate passes
  (16 changed files, none under `dart/`, `python/`, `cmake/`, `CMakeLists.txt`,
  packaging, `.github/`, or `pixi.lock`); 37/37 bypass attempts rejected with
  the unmodified tree validating at zero errors as a control; every
  antisymmetry number reproduces exactly from `raw_rows`; the typed-unsupported
  reasons re-verified against `dart/constraint/` on this head. Findings acted
  on:
  1. MAJOR — the packet attested to assertions and a `target.commit` its
     recorded run predated, because writer changes landed without regenerating.
     The packet has been regenerated on the current writer. Its `raw_rows` are
     byte-identical across regenerations because the fixture is deterministic,
     which the packet itself evidences through
     `raw_rows[*].repeat_trajectory_sha256`.
  2. MINOR — `raw_paths` accepted a directory (`.exists()`), negative controls
     in a subdirectory were enumerated by neither loop, a non-string entry in a
     lane's `evidence` list was silently dropped (a genuine bypass: a lane could
     be closed by `[{"path": ...}]`), and `scene.digest` was format-checked but
     never recomputed. All four are fixed and covered by new tests (52 cases).
  3. Residual gap recorded, not fixed: the validator performs no arithmetic
     cross-check of derived metrics against `raw_rows`, so a falsified summary
     would pass. That is a WS4-scale change (the gate would need to know each
     packet's derivation) and is listed as follow-up rather than attempted here.
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

## Codex review round 1 (PR #3444) — 2026-08-16

Codex posted 4 reviews with 15 inline findings (P1/P2); all addressed in
this round, fixes silent per repo convention (no thread replies):

- Validator fail-closed gaps (ported identically to `main`): closed-lane
  disposition must match the packet's `result.disposition`; the two-review
  floor requires DISTINCT reviewers; review passes must carry a
  `content_digest` binding them to the packet content they reviewed
  (regeneration invalidates); `raw_rows` entries must be non-empty
  structured records; `raw_paths`/`visual` paths must be relative,
  non-escaping, and resolve inside approved roots; sweep/seed ensembles
  need valid DISTINCT entries; string metric leaves are allowed only under
  semantic keys (prose can no longer masquerade as a measurement);
  `configuration.requested/resolved` must carry a recognizable identity
  key with no null values; NaN/Infinity are rejected at JSON load; lane
  evidence paths are normalized before indexing (no dual-spelling
  owner/review bypass) and may not escape the sidecar;
  `corpus_reference` is pinned to the canonical DART 7 corpus path and
  branch; `target.fetch_hint` is required so target commits stay
  reproducible after squash-merge (PR head refs survive).
- `preserve_review` now binds carried passes to the regenerated packet's
  content digest (stale reviews are dropped, failing closed);
  `record_review_pass` added for digest-bound review recording.
- CT-001 writer: the energy-gain gate now starts from the launched
  (pre-step) state so the first contact solve is covered, and the claim
  boundary/limitations are derived from the computed detector sets
  (`anisotropic_detectors`, `nonconforming_detectors`,
  `identical_detector_groups`) instead of hardcoding one historical run's
  bullet statistics. Packet regenerated: disposition `reproduced`
  unchanged; bullet still excluded, now by computed criteria.
- Design doc: the ordered "Prioritize" list moved out of the durable
  document (ordering is mutable task state; the doc now records the row
  set and rationale only), and the document was added to the
  `docs/design/README.md` index.
- Validator test suite: 52 -> 65 cases (every new rejection has a test;
  closed-lane tests rewritten for digest-bound, distinct-reviewer,
  disposition-matching closure).
- `pixi run check-citation-evidence` passes on the real tree; the
  permanent negative control keeps failing (error count grew under the
  stricter rules).

## Codex review round 2 (PR #3444) — 2026-08-16

Round 2 re-tested the round-1 fixes adversarially; 4 new findings, all
fixed (rules mirrored on main): scene.parameters required (a digest
without published content binds nothing); exhaustive metric-leaf type
chain with booleans accepted explicitly as measured findings; reviewer
identities normalized (strip+casefold) before the distinct-reviewer
closure count; visual entries must carry a recognized media suffix. The
CT-001 packet was regenerated at the round-2 commit `1a811de5bbb` so its
recorded command runs the current writer at its recorded, fetchable
target. Disposition unchanged (reproduced). Tests: 65 -> 69; tree
validates; negative control still fails.

## Codex review round 3 (PR #3444) — 2026-08-16

Three findings, all fixed on both branches: `measurement_window` is
structurally validated (non-empty string, or a non-empty object of finite
numeric values with `start_s <= end_s`; truthy placeholders rejected);
visual artifacts are verified by media SIGNATURE, not filename — a prose
file renamed to `capture.png` is rejected by magic-byte check
(PNG/JPEG/GIF/WebP/SVG/MP4/WebM); configuration identity values must be
non-empty strings. Review passes additionally require an explicit
`verdict: "pass"` (mirrored from the main round-3 finding). Tests:
69 -> 73.

## Codex review round 4 (PR #3444) — 2026-08-16

Two findings, both fixed: not-applicable lanes can no longer publish a
disposition or hold evidence packets (an early `continue` had skipped all
consistency checks for them), and the Windows `check-lint` task variant
now includes `check-citation-evidence` so the gate runs on the Windows CI
lane too. Shared round-4 rules mirrored from main: metadata-suffixed keys
(method_note, backend_reason) no longer count as configuration
identities; raw rows and metric groups must carry at least one
numeric/boolean measurement or typed-unsupported marker — metadata-only
records cannot satisfy the raw-evidence or measured-group requirements.
Tests: 73 -> 77.

## Codex review round 5 (PR #3444) — 2026-08-16

Four findings, all fixed (mirrored on main): asserted deterministic
repeats require the recorded `deterministic_repeats_identical: true`
verification flag; measurement windows must be structured numeric objects
(prose strings rejected); visual artifacts are validated as structurally
complete media containers (begin and end markers plus minimum size; full
decode would need an image dependency — recorded boundary); fetch hints
must match the durable PR-ref fetch form (reachability is guaranteed by
GitHub PR head refs, checked at write time via --freshness — recorded
boundary). Identity keys also match whole tokens now. Tests: 77 -> 81.

## Codex review round 6 (PR #3444) — 2026-08-16

Two findings here plus four shared, all fixed (mirrored with main):
exact-form fetch hints (fullmatch of the fetch-and-checkout command);
named measurement-window bounds; placeholder identity values rejected;
header+tail media reads (no false truncation above 8 MiB); negative
controls pinned defect-by-defect via .expected-errors.json sidecars
(CT-019 sidecar added). Tests: 81 -> 84.

## Codex review round 7 (PR #3444) — 2026-08-16

Three findings here plus four shared, all fixed (mirrored with main):
runnable fetch hints embedding the packet's own commit; orphan sidecars
no longer count as negative controls; step-window bounds validated as
sane integers; reproducible-form commands; hash-bound repeat claims;
bookkeeping-only rows rejected. The CT-001 hint was migrated in place
(content-equal to the updated writer's emission). Tests: 84 -> 90.

## Codex review round 8 (PR #3444) — 2026-08-16

Three findings here plus three shared, all fixed (mirrored with main):
open-lane dispositions must match their packets (null allowed,
contradiction not); shell tails rejected in reproduction commands;
duplicate JSON keys rejected at load; raw-data suffix whitelist for
raw_paths; digest-shaped hash values; http(s) source URLs. Tests:
90 -> 96.

## Codex review round 9 (PR #3444) — 2026-08-16

Three findings here plus the shared newline defect, all fixed (mirrored
with main): newline-proof command validation (a genuine round-8 regex
defect); per-repeat hash lists for repeat counts above 2; structural
parsing of raw-data artifacts. Tests: 96 -> 100.

## Codex review round 10 (PR #3444) — 2026-08-16

Two findings here plus two shared, all fixed (mirrored with main):
declared sweep/seed points must be matched by recorded rows; the broken
DART_CITATION_PR override removed (ownership transfer updates writer
and validator constants together); artifact_digests bind path-based
evidence bytes; the CT-011 full-state instrument fix is main-only (no
6.20 restore packet). Tests: 100 -> 102.

## Codex review round 11 (PR #3444) — 2026-08-16

Three findings here plus two shared, all fixed (mirrored with main):
clean-tree enforcement in writers; sweep-point/seed observation
binding; JSON artifact content requirements; and the instrument sweep
fixed this branch's CT-001 NaN-blindness (validity gates would pass a
NaN trajectory since NaN comparisons are False — rows now carry an
explicit full-state finite flag gated in validity_failures). CT-001
regenerated at the round-11 commit. Tests: 102 -> 105.

## Codex review round 12 (PR #3444) — 2026-08-16, loop checkpoint

Three findings here plus two shared, all fixed (mirrored with main):
scalar sweep-point observation binding; sweep/seed ensembles require
inline rows; required host provenance with explicit performance_valid;
numeric-content CSV validation; WebP/MP4 container consistency. The
review loop pauses at this checkpoint for the maintainer's decision;
every posted finding through round 12 is addressed and pushed. Tests:
105 -> 110.

## Pre-merge gate at the loop checkpoint — 2026-08-16

Full `pixi run test-all` on the checkpoint head `886558f28f1`: exit 0,
all phases pass (158/158). This is the pre-merge evidence for PR #3444;
the merge decision (or a continue-the-loop call) rests with the
maintainer.

## Codex review round 13 (PR #3444) — 2026-08-16, loop resumed by maintainer

Two findings here plus three shared, all fixed (mirrored with main):
structural NPY/NPZ/parquet parsing; WebM DocType verification;
NFKC-normalized reviewer identities; object-only sweep points. Tests:
110 -> 113.

## Codex review round 14 (PR #3444) — 2026-08-16

One finding here plus shared fixes (mirrored with main): evidence/raw/
is reserved for raw-data artifacts — excluded from packet discovery and
un-referenceable by lanes. This branch's CT-001 already hashed per
step, so no regeneration was needed. Tests: 113 -> 114.
