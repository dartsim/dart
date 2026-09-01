# Handoff - DART 6 deformable body feature and performance

Updated: 2026-08-01

## Status

The representative DART 6 deformable-body performance and soft-contact slice
is merged:

- #3382 merged as `6c88ac1d774a702b494643fb598be6b8af9385e1`;
- #3381 established the single built-in `dart` collision detector;
- #3407 removed the out-of-scope volumetric-FEM subsystem from DART 6;
- #3408 + #3423 shipped PR-3a soft-foot SIMBICON (merged as
  `fe9cb9ebd73b176794df2de7179f5d23f146cbe6` and
  `73cd91e69dba635cb17e93e60f33a2c245e629d0`). The contact-spreading claim
  is gate-asserted (51.2 soft vs 15.64 rigid contacts); #3431 then
  re-measured push recovery with replica ensembles — the single-trajectory
  18000 N was a resonance pocket, and no measured configuration shows a
  soft advantage (rigid ahead clean and on the noisy floor, tied under 20%
  motor noise): the paper's soft-advantage ordering is an open gap (see
  `12-pr3a-soft-foot-simbicon.md` "Robustness re-measurement").

PLAN-622 remains active for the Jain/Liu point-mass surface model. Do not
resume any branch used by the merged work.

## Durable owners

- Architecture and compatibility:
  `docs/design/dart6_deformable_body.md`
- Paper targets:
  `docs/background/deformable_body_paper_targets.md`
- Current priority and gates:
  `docs/plans/dashboard.md`
- Live claim status:
  `docs/dev_tasks/dart6_deformable_body_performance/02-paper-parity-matrix.md`
- Accepted merged-slice verification:
  `docs/dev_tasks/dart6_deformable_body_performance/verification.md`

## Exact next action

Work the task to full completion for the DART 6.20 release (maintainer
direction, 2026-08-01), bundling related items into as few PRs as review
quality allows. Parity rows close only with gate evidence (the matrix
allows them no deferral); an approved disposition is an alternative only
where an item's own acceptance text offers one (WP-DB.07's negative
disposition, the paired-benchmark disposition). `RESUME.md` holds the
ordered item list and takeover detail. Completed packet specs such as
`12-pr3a-soft-foot-simbicon.md` are records, not resumable work.

## Open acceptance work

- Close the push-recovery mechanism gap (#3431 built the motor-noise
  variant and the noisy-floor surface with robust ensemble gates; no
  measured configuration shows a soft advantage — suspects: controller
  adaptation to soft contact, SoftContactConstraint solve quality under
  impulsive load).
- Build the remaining Jain/Liu scene rows: soft-contact walk (which also
  unblocks the noisy-floor row's course-tracking outcome), hand/arm
  manipulation (finger flick, arm fold, pinch grasp), and the four-link
  flexible-foot comparison.
- Apply the approved competitive envelope (`decisions.md` item 2, confirmed
  2026-07-23: in-tree backends plus normalized paper metrics) to the
  performance-acceptance evidence; the definition itself is closed.
- Complete the four-link flexible rigid-foot versus deformable-foot
  comparison.
- Resolve WP-DB.07 multicore scaling with representative evidence or an
  explicit negative disposition.
- Complete the `dart` detector coverage, allocation, determinism, and
  same-host performance gates required before any default proposal.
- Produce a complete `bm-soft-body-paired` artifact or record an approved
  disposition.
- Land the zero-DoF soft point-mass assertion fix separately on `main`.

## Guardrails

- Preserve C++17, pybind11, installed DART 6 APIs, ABI-sensitive layouts,
  default simulation behavior, and Gazebo/gz-physics compatibility.
- Keep adaptive contact activation and soft face-interior coverage opt-in
  unless a separately approved packet changes their defaults.
- Do not restart the volumetric-FEM subsystem on `release-6.20`.
- Do not propose making `dart` the default collision detector from this task
  without satisfying the durable pre-default contract.
