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

## CT-004 articulated energy slice — 2026-08-14

- What changed: `scripts/write_citation_ct004_articulated_energy_packet.py`
  and its packet; manifest lane, dashboard, changelog, and task status.
- Scene: passive 4-link planar pendulum chain (revolute hinges about y, 0.3 m
  links, 1 kg each) released from rest at alternating 0.35 rad joint angles,
  no contact and no control, 2 s horizon. A passive chain conserves total
  mechanical energy exactly, which makes the oracle solver-neutral.
- Commands and results:
  - `PYTHONPATH=build/default/cpp/Release/python pixi run python
scripts/write_citation_ct004_articulated_energy_packet.py` — 8 cells
    (2 families x 4 timesteps), 2 deterministic repeats each, all
    bit-identical; integration family, timestep, and gravity each asserted
    against the request per repeat and per cell.
  - Measured relative energy drift as dt goes 4 ms -> 0.5 ms:
    SEMI_IMPLICIT 5.675e-1 -> 6.471e-2, VARIATIONAL 8.697e-2 -> 1.051e-2.
    Observed least-squares log-log slopes 1.04 and 1.02.
  - `pixi run check-citation-evidence` — OK; 68 pytest cases pass.
- Disposition `reproduced` covers only the convergence half of the corpus
  row. The "at matched cost" half is explicitly uncovered: no timing
  methodology was applied, `metrics.performance` is typed unsupported, and no
  ranking between the two families is claimed even though their drift figures
  differ.
- Angular momentum is recorded as an observed envelope, not a conservation
  oracle, because gravity exerts a torque about the world origin.
- Negative control: unchanged and still failing.

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

## Final independent review and third fix pass — 2026-08-14

A fresh reviewer ran 44 adversarial bypass attempts against the current head
(all rejected), re-verified the three `citation_packet_utils.py` source claims
against the tree, and confirmed the negative control trips 17 errors. Two
majors and four minors were found and acted on:

1. MAJOR — a genuine gate bypass: a non-string entry in a lane's `evidence`
   list was silently dropped, so `evidence: [{"path": ...}]` on a `closed`
   lane validated with zero errors, skipping the missing-packet, not-JSON,
   outside-`evidence/`, negative-control, and two-review checks. This is the
   same "drop what you don't understand" shape as the round-1 subdirectory
   bypass. Non-string entries now fail, with a test covering five variants.
2. MAJOR — CT-004's disposition was `reproduced` on an oracle unrelated to
   the cited claim. The corpus defines `reproduced` as "the claimed behavior
   is observed"; CT-004's claim is a methodological prescription (compare at
   matched cost) and this packet measures no cost, so none of the claim's
   content was observed. The row is now `unresolved`, following the CT-003
   precedent, and the convergence result it did establish is published as
   `metrics.physical.convergence_finding` with an explicit note that it does
   not promote the row.
3. MINOR — `raw_paths` accepted a directory (`.exists()` -> `.is_file()`), and
   `scene.digest` was format-checked but never recomputed from the parameters
   the packet publishes (now verified; a hand-edited scene fails).
4. MINOR — CT-004's relative drift divides by an initial energy containing a
   datum-dependent gravitational term, so the headline percentage is
   gauge-dependent while the slope and verdict are not. Disclosed, with the
   absolute drift published per row.
5. MINOR — CT-001's claim boundary stated parameters only; it now leads with
   the observed outcome like the other three. CT-002 publishes the basis of
   its energy tolerance rather than a bare number.
6. MINOR — every packet shipped `review.passes: []` hardcoded, so recorded
   reviews could not survive regeneration. Writers now carry forward an
   existing review block, which is what makes the two-review closure floor
   reachable without hand-editing a generated file.

Recorded as follow-up, not fixed: the validator does not arithmetically
cross-check derived metrics against `raw_rows`, so a falsified summary would
pass. Closing that needs the gate to know each packet's derivation, which is
WS4-scale work.

- Commands after this pass: `pixi run check-citation-evidence` — OK;
  `tests/test_check_citation_evidence.py` — 72 passed.

## CT-005 PD-tracking slice — 2026-08-14

- What changed: `scripts/write_citation_ct005_pd_tracking_packet.py` and its
  packet; manifest lane, dashboard, changelog, task status.
- Scene: 4-link chain hanging under gravity, links 0.3 m / 1.0 kg with mass at
  mid-link, tracking a 0.5 Hz / 0.25 rad phase-shifted sinusoidal joint
  reference, no contact, 2 s horizon.
- Four tuning iterations were needed and are worth recording, because the
  first three produced a fixture that measured the controller rather than the
  integrator: a fixed-gain diagonal PD saturated its 500 Nm limit in every
  cell and diverged at the coarser timesteps. The diagnosis, from a direct
  probe rather than another guess, was that the root joint of a serial chain
  sees a far smaller articulated inertia than the composite rigid-body
  inertia used to size its gains, so root gains stiff enough to hold the
  chain were unstable while the tip tracked fine. Replaced with a
  computed-torque controller built on `Multibody.compute_inverse_dynamics`,
  which gives every configuration the same closed-loop bandwidth. Result:
  no saturation, max torque 143.7 Nm, RMS error about 1% of the reference
  amplitude.
- Measured RMS tracking error and control work, per family, dt 4 ms -> 0.5 ms:
  SEMI_IMPLICIT 2.138e-3 -> 2.412e-3 rad with work 6.086 -> 6.534 J;
  VARIATIONAL 2.139e-3 -> 2.412e-3 rad with work 6.085 -> 6.534 J.
- Finding: the error is controller-limited, not integration-limited.
  Refining the timestep eightfold does not reduce tracking error (it varies
  by ~13% and is slightly lower at the coarsest step) while control work
  rises monotonically. That is a substantive result about the row's premise:
  the whole-step tradeoff is not simply "smaller timestep is more accurate".
- Disposition `unresolved`: the cited claim is a speed/accuracy tradeoff and
  no step cost is measured, so the row is not promoted. `constraint_error` is
  typed unsupported because this scene has no constraints to violate.
- Commands: packet writer as recorded in the packet;
  `pixi run check-citation-evidence` — OK; 72 pytest cases pass.

## WS4 first slice: resolved configuration in Python — 2026-08-14

- What changed: nanobind bindings for `ResolvedConfigurationNote` and
  `ResolvedSolverConfiguration` (`python/dartpy/simulation/module_compute.cpp`),
  the `World.resolved_configuration` property
  (`python/dartpy/simulation/module_world.cpp`), surgical stub entries, two
  Python tests, a `world_resolved_configuration()` helper, and all five
  packets regenerated to record it.
- Why it mattered: every packet had been recording resolved identity by
  reading back the option it had just set, which cannot distinguish a method
  that ran from one that was silently substituted. Three review rounds
  flagged this as the blocking gap behind the typed-unsupported markers.
  Packets now carry, per domain, the World's own bake-time decision:
  requested, resolved, reason, and a substitution flag.
- Verified against a live World: before `enter_simulation_mode` the
  configuration is empty; after bake it reports four domains
  (rigid-body, rigid-contact, multibody, deformable-psd), and requesting
  BOXED_LCP versus SEQUENTIAL_IMPULSE changes the recorded `rigid-contact`
  resolution accordingly.
- Scope discipline: `pixi run generate-stubs` rewrote 1990 lines across eight
  stub files, because the committed stubs were already stale relative to the
  build. That churn was reverted and the 37 lines corresponding to this
  change were added by hand, keeping the diff surgical; the pre-existing stub
  drift is left for whoever owns it rather than being swept in here.
- Commands: `pixi run build` — success; the two new tests pass;
  `pixi run check-dartpy-import-layout` — passed;
  `pixi run check-docs-policy` — passed.
- Remaining WS4 gap: no comparable per-solve residual exists, so
  `metrics.numerical.solver_residual` stays typed unsupported in every packet.

## CT-007 high-mass-ratio slice — 2026-08-14

- Scene: two-box stack at rest (0.2 m cubes, 1 kg lower box), upper mass swept
  over four decades so the loaded contact sees ratios 1:1 to 1000:1, both
  contact solvers, dt 2 ms, 2 s horizon, two deterministic repeats per cell.
- Disposition `unresolved`: the cited claim compares against exact-cone
  methods and no exact-cone arm exists on this branch, so the row cannot be
  reproduced. The packet establishes the baseline arm the WS5 GO/NO-GO needs.

### Finding worth a maintainer decision

`SEQUENTIAL_IMPULSE`, the World's default contact solver, fails completely at
mass ratios 100 and 1000:

| solver             | ratio 1 | ratio 10 | ratio 100  | ratio 1000 |
| ------------------ | ------- | -------- | ---------- | ---------- |
| SEQUENTIAL_IMPULSE | 1.41e-4 | 1.02e-3  | **1.0000** | **1.0000** |
| BOXED_LCP          | 1.96e-4 | 1.98e-4  | 3.26e-5    | -3.01e-4   |

(relative closure of the loaded contact: 1.0 means the two box centers have
closed by a full box height.)

At ratios 100 and 1000 the heavy box descends 0.20002 m -- exactly one box
height -- while the lower box moves by microns (2.4e-6 and 5.3e-6 m), and the
pair comes to rest fully interpenetrated at near-zero velocity (4.8e-7 and
3.5e-6 m/s). The upper box passes through the lower one rather than the pair
sinking into the ground. Both outcomes are bit-identical across repeats.
`BOXED_LCP` holds the same stack across all four decades.

This is consistent with the known behavior of iterative Gauss-Seidel contact
at high mass ratio under a fixed iteration budget, which is what makes it
credible rather than a fixture artifact. The packet does not claim the
mechanism: confirming it needs the per-solve residual WS4 has not exposed.
The scene rests each box exactly on the one below, so the initial overlap is
zero and the observed closure is produced by the solve.

- Commands: packet writer as recorded in the packet;
  `pixi run check-citation-evidence` — OK (it rejected an untyped `None` for
  the "no failure onset" case on the first attempt, which is now a typed
  unsupported marker).

## CT-011 restore-equivalence slice — 2026-08-15

- Scene: five-sphere pile with a 0.5 s contact-rich warm-up; seven protocol
  arms per contact solver, every continuation hashed bit-exactly over the
  full state vector; whole protocol repeated twice and required identical.
- The finding was pinned by four escalating probes before the packet was
  written, each ruling out an explanation:
  1. In-place `state_vector` restore diverges from the original continuation
     at the FIRST post-restore step (max state delta ~5e-3 to 1e-2 within a
     0.2 s window), both solvers.
  2. Ballistic control is bit-exact (protocol and state vector are sound for
     free motion), and `update_kinematics()` after restore changes nothing.
  3. Two in-place restores of the SAME snapshot differ from each other when
     different history precedes them -- but two FRESH worlds restoring that
     snapshot agree bit-exactly, two worlds with IDENTICAL histories agree,
     and a world whose history ends before first contact matches fresh.
  4. Deactivation ruled out: no body was asleep at the snapshot and
     disabling deactivation changes nothing.
- Conclusion: once a World has contact history, the post-restore trajectory
  is a function of (restored state, prior contact history), not of the
  restored state alone. Everything is deterministic given full history; this
  is hidden result-affecting contact state surviving the restore, not
  nondeterminism. This is precisely the design doc's requirement that
  "reset semantics must explicitly choose whether solver/contact history is
  preserved", demonstrated unmet/undocumented on `main`, and it is the
  motivating evidence for the WS3 contact-identity work.
- Practical workaround established by the packet: restore into a freshly
  built world -- bit-exact and repeatable.
- Disposition `unresolved`: CT-011 is a requirements claim (research
  workflows need X), which a fixture cannot reproduce; reset cost,
  allocation, and concurrency are typed unsupported.
- The gate rejected `first_divergent_step: 0` until it was declared a
  measured zero -- which it genuinely is: divergence starts at the very
  first step.
- Commands: packet writer as recorded in the packet;
  `pixi run check-citation-evidence` — OK.

## External mutations — 2026-08-15

With explicit maintainer approval ("go ahead" on posting the issues, pushing
both branches, and opening the two PRs):

- Posted issue #3442 (CT-007: SEQUENTIAL_IMPULSE lets a heavy box sink
  through a light one at mass ratios >= 100) and issue #3443 (CT-011:
  state-vector restore depends on prior contact history), from the drafts in
  this folder.
- Merged the moved bases into both branches before pushing (lockfile-only
  commits #3441 on `main`, #3440 on `release-6.20`), per the merge-first
  push rule; `test-all` re-run on both merged heads before the push.

## CT-011 root cause identified — 2026-08-15

The WS3 mechanism hunt closed in three probes against the rebuilt tree:

1. At the identical restored state, in-place and fresh worlds report the same
   contact pairs, depths, world points, and normals -- only `local_point_a`
   differs, and the in-place value is a radius-length vector pointing away
   from the bottom pole: a body-frame anchor of a ROTATED sphere.
2. Reading `World::getStateVector()` shows the design directly: per dynamic
   rigid body it stores `transform.position` and `velocity.linear` only. The
   header documents the dense order as "dynamic rigid-body translations"
   (built for the differentiable rigid-body path).
3. The decisive probe: `state_vector` has 30 entries for the five-sphere
   scene (3 pos + 3 linvel each); after restoring the same vector, the
   in-place sphere keeps a rolled orientation (Frobenius deviation 2.41 from
   identity) and ~7 rad/s angular velocity while the fresh sphere sits at
   identity with zero spin.

So the CT-011 divergence is a PARTIAL RESTORE by design, not hidden contact
state: orientation and angular velocity are never captured, a restored world
keeps whatever rotational state it had, and friction couples that into the
continuation. Contact history mattered only because contact is what makes
spheres spin -- the earlier "shared contact-pipeline state" hypothesis in
issue #3443 was wrong in mechanism (the behavioral facts and repro all
stand) and the issue body was corrected with maintainer approval as part of
the approved posting workflow. The packet limitation now states the root
cause and the full-state alternatives (save_binary/load_binary,
restore_replay_frame), neither of which is a lightweight in-memory reset.

The dead-end candidates are recorded because ruling them out was real work:
persistent_manifold_cache.cpp (warm-start impulses, classic dart detector
facade -- not on the DART 7 World query path) and deactivation state (probed,
nothing asleep, disabling changed nothing).

## Codex review round 1 (PR #3445) — 2026-08-16

Codex posted 4 reviews with 14 inline findings (P1/P2); all addressed in
this round, fixes silent per repo convention (no thread replies):

- Validator fail-closed gaps (mirrored on release-6.20): closed-lane
  disposition must match the packet's `result.disposition`; the two-review
  floor requires DISTINCT reviewers; review passes must carry a
  `content_digest` binding them to the packet content they reviewed;
  `raw_rows` entries must be non-empty structured records;
  `raw_paths`/`visual` paths must be relative, non-escaping, and resolve
  inside approved roots; sweep/seed ensembles need valid DISTINCT entries;
  string metric leaves are allowed only under semantic keys; requested/
  resolved configurations must carry a recognizable identity key with no
  null values; NaN/Infinity rejected at JSON load; lane evidence paths
  normalized before indexing; `target.fetch_hint` required (PR head refs
  survive squash-merge, keeping recorded commits reproducible — all seven
  packet target commits verified as ancestors of the PR head).
- `preserve_review` binds carried passes to the regenerated packet's
  content digest; `record_review_pass` added; all seven writers rebind
  after assembly and emit `target.fetch_hint`.
- CT-011 writer: the state-function aggregate now includes EVERY protocol
  arm (fresh-world, cross-history, pre-contact, ballistic — not just the
  two in-place booleans), and the writer aborts with an explicit error if
  a future branch makes every arm bit-exact, instead of regenerating the
  divergence narrative against contradicting measurements. Packet
  regenerated; findings unchanged (divergence still observed, disposition
  `unresolved`).
- Dev-task docs: RESUME no longer instructs unconditional `@codex review`
  re-triggering (scoped to approved post-fix rounds per the
  external-mutation policy); README's next-steps no longer tell a resumed
  agent to re-implement the already-landed `ResolvedSolverConfiguration`
  bindings (only the per-solve residual remains WS4 work).
- Validator test suite: 74 -> 85 cases; existing packets migrated with
  `fetch_hint`; `pixi run check-citation-evidence` passes on the real
  tree; the permanent negative control keeps failing with a higher error
  count under the stricter rules.

## Codex review round 2 (PR #3445) — 2026-08-16

Round 2 re-tested the round-1 fixes adversarially; 3 new findings, all
fixed (mirrored where shared): scene.parameters is now required so a
digest cannot bind to absent content; the metric-leaf type chain is
exhaustive (booleans accepted explicitly as measured findings, anything
else rejected); reviewer identities normalized before the
distinct-reviewer count; visual entries must be recognized media; the
CT-011 writer asserts its EXACT measured arm pattern and aborts naming
any deviating arm; RESUME's unsupported-quantities list no longer names
the landed ResolvedSolverConfiguration binding. The CT-007 finding
(writer absent at its recorded target commit) was generalized: ALL seven
packets regenerated at the round-2 commit `f4e9e5c11d5`, whose tree
contains every current writer, so each recorded command plus fetch_hint
reproduces its packet from a clean checkout. All dispositions unchanged
(CT-001/002 reproduced; CT-003/004/005/007/011 unresolved). Tests: 85 ->
89; tree validates; negative control still fails.

## Codex review round 3 (PR #3445) — 2026-08-16

Two findings, both fixed on both branches: identity keys must carry
non-empty string values (`{"solver_method": ""}` no longer counts as a
recorded identity), and review passes must record an explicit
`verdict: "pass"` — a digest-bound entry summarizing a FAILED review can
no longer count toward the two-review closure floor
(`record_review_pass` stamps the verdict). Tests: 89 -> 93.
