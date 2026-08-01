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
  `73cd91e69dba635cb17e93e60f33a2c245e629d0`. Both Jain/Liu biped claims now
  reproduce and are gate-asserted: contacts 51.2 soft vs 15.64 rigid,
  recoverable push 18000 N soft vs 8000 N rigid
  (`12-pr3a-soft-foot-simbicon.md` "Resolved decisions").

Re-fetch before starting: the release branch advances frequently, and the
commit above is a snapshot rather than a permanent branch tip.

## Immediate packet

PR-3a is complete (#3408 + #3423 above). No next packet is preselected:
reassess `docs/plans/dashboard.md` (PLAN-622) and pick from the open
items with fresh evidence rather than assuming an order. Known open items:

- Motor-noise variant of the push-recovery comparison (the remaining
  unreproduced clause of the Jain/Liu biped row; see
  `02-paper-parity-matrix.md`).
- Competitive-envelope definition.
- Four-link flexible-foot comparison.
- WP-DB.07 scaling; WP-DB.08 DART-owned/pre-default collision coverage.
- A valid `bm-soft-body-paired` artifact or an approved disposition.
- The separate `main` PR for the zero-DoF soft point-mass assertion.

For whichever packet is chosen: fetch `origin/release-6.20`, create a fresh
non-tracking topic branch, implement the smallest complete slice with
text-first then visual evidence, and run the focused and downstream gates
before publishing.

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
