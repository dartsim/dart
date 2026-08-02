# RESUME - DART 6 deformable body feature and performance

Updated: 2026-08-01

## Start here

Read, in order:

1. `docs/ai/principles.md`
2. `README.md`
3. `docs/design/dart6_deformable_body.md`
4. `docs/background/deformable_body_paper_targets.md`
5. `02-paper-parity-matrix.md`
6. `verification.md`

Use `docs/plans/dashboard.md` for current priority. Treat older commits and
merged PR branches as evidence only, not as resumable work.

## Verified merged state

- #3382 merged into `release-6.20` on 2026-07-29 as
  `6c88ac1d774a702b494643fb598be6b8af9385e1`.
- #3381 merged the DART-owned collision implementation into the built-in
  `dart` detector as `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.
- #3407 removed the volumetric-FEM subsystem from DART 6 as
  `2ffe228c14c67e120d2a946ce9d36e8a9658044f`.
- The DART 6 task therefore covers the Jain/Liu point-mass surface model only.
- #3408 merged PR-3a soft-foot SIMBICON (scene, comparability gates, push
  sweep) as `fe9cb9ebd73b176794df2de7179f5d23f146cbe6`.
- #3423 resolved the PR-3a maintainer decisions (matched control rest inertia,
  asset `<damp>` 1000 -> 4000, point-mass-aware SIMBICON COM sensor) as
  `73cd91e69dba635cb17e93e60f33a2c245e629d0`. The contact-spreading claim
  is gate-asserted: contacts 51.2 soft vs 15.64 rigid
  (`12-pr3a-soft-foot-simbicon.md` "Resolved decisions").
- #3431 re-measured push recovery with phase-strided replica ensembles and
  corrected the record: the single-trajectory "18000 N soft" was an
  isolated resonance pocket; robust thresholds (rigid vs soft) are
  8000/4000 N clean, 4000/4000 under 20% held motor noise, 6000/2000 on
  the paper's 2 cm fold-free jittered-mesh floor (two earlier higher rigid
  readings were review-caught collision-geometry artifacts). The paper's soft-advantage push
  ordering is an **open gap** (`12-pr3a-soft-foot-simbicon.md`
  "Robustness re-measurement"); gates protect the measured floors and
  print the response curves.

Re-fetch before starting: the release branch advances frequently, and the
commit above is a snapshot rather than a permanent branch tip.

## Immediate packet

PR-3a is complete (#3408 + #3423 above). The maintainer-set goal
(2026-08-01) is to **fully complete this task for the DART 6.20 release**,
bundled into as few PRs as review quality allows. The parity rows (items
1-5) close only with gate evidence — the matrix's acceptance rule allows
them no deferral. An explicitly approved disposition is an alternative only
where an item's own acceptance text offers one (item 7's negative
disposition and item 9's disposition).

Remaining items (parity rows from `02-paper-parity-matrix.md` and
`10-full-parity-execution-plan.md` section 5, plus acceptance work):

1. Motor-noise variant of the push-recovery comparison — **built in
   #3431** (robust thresholds at 20% held noise); what remains of the row
   is the mechanism gap: no soft advantage in any measured configuration
   (controller adaptation to soft contact and SoftContactConstraint solve
   quality under impulsive load are the recorded suspects).
2. Noisy-floor biped row — the seeded jittered-mesh floor and an adjacent
   push-recovery gate are **built in #3431**; the row's own
   course-tracking outcome is blocked on item 3's walking gait.
3. Biped soft-contact walk row (SIMBICON walk; LCP cadence per the paper).
4. Four-link flexible-foot comparison row (same controller and seed).
5. Hand/arm models and manipulation rows (finger flick, arm fold, pinch
   grasp).
6. Apply the competitive envelope to the performance-acceptance evidence.
   The definition itself is closed: `decisions.md` item 2 (in-tree backends
   plus normalized paper metrics) was confirmed by the maintainer
   2026-07-23 — do not reopen it.
7. WP-DB.07 multicore scaling: representative evidence or an approved
   negative disposition.
8. WP-DB.08 pre-default `dart` detector gates (coverage, allocation,
   determinism, same-host performance).
9. A complete `bm-soft-body-paired` artifact or an approved disposition.
10. The separate `main` PR for the zero-DoF soft point-mass assertion.

Suggested bundles to keep the PR count low: the biped rows (1-4) in at most
two scene/gate PRs; the hand/manipulation rows (5) as one PR; performance
and closure evidence (6-9) as one PR; item 10 stays a separate `main` PR by
policy.

For each packet: fetch `origin/release-6.20`, create a fresh non-tracking
topic branch, implement the smallest complete slice with text-first then
visual evidence, and run the focused and downstream gates before
publishing.

## Required evidence

- Finite state and deterministic final-state metrics.
- Rigid-foot versus deformable-foot comparison using the same controller and
  initial conditions.
- Contact, support, and controller-stability observations tied to explicit
  pass/fail thresholds.
- One-thread and host-capped multi-thread measurements when performance is
  claimed.
- A reproducible `dart-demos` command plus captured visual evidence.
- Gazebo/gz-physics coverage for collision, constraint, or default-policy
  changes.

Do not count interrupted benchmark directories as evidence. A paired-runner
artifact is complete only when its completion marker and all required raw rows
exist.

## Verification commands

Run the exact focused tests named by the packet, then at minimum:

```bash
pixi run lint
pixi run check-lint
pixi run build
pixi run test
pixi run -e gazebo test-gz
```

Use `dart-verify-sim` for scene behavior and visual claims. Cap local build
parallelism when the host is shared.

## Stop conditions

- Do not restart Kim/Pollard volumetric FEM on DART 6.
- Do not change public layouts, defaults, or installed compatibility surfaces
  without explicit approval and the corresponding acceptance evidence.
- Do not make `dart` the default detector from this packet.
- Keep the `main` zero-DoF soft point-mass assertion fix in a separate PR.
- GitHub mutations, pushes, review requests, and branch deletion require the
  authorization defined by the repository instructions.
