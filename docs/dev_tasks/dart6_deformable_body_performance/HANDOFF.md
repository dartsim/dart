# Handoff - DART 6 deformable body feature and performance

Updated: 2026-07-30

## Status

The representative DART 6 deformable-body performance and soft-contact slice
is merged:

- #3382 merged as `6c88ac1d774a702b494643fb598be6b8af9385e1`;
- #3381 established the single built-in `dart` collision detector;
- #3407 removed the out-of-scope volumetric-FEM subsystem from DART 6.

PLAN-622 remains active for the Jain/Liu point-mass surface model. Do not
resume any branch used by the merged work.

## Durable owners

- Architecture and compatibility:
  `docs/design/dart6_deformable_body.md`
- Paper targets:
  `docs/background/deformable_body_paper_targets.md`
- Current priority and gates:
  `docs/plans/dashboard.md`
- Active packet:
  `docs/dev_tasks/dart6_deformable_body_performance/12-pr3a-soft-foot-simbicon.md`
- Accepted merged-slice verification:
  `docs/dev_tasks/dart6_deformable_body_performance/verification.md`

## Exact next action

Implement PR-3a soft-foot SIMBICON from a fresh non-tracking topic branch
based on the explicitly authorized release tip. Reuse the existing
`atlas_simbicon` controller and soft-feet Atlas asset. Keep the scene in
`dart-demos`; do not add a standalone GUI executable.

## Open acceptance work

- Define and approve the competitive implementation envelope.
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
