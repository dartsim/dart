# DART 6 deformable-body parity and performance

The unmet DART 6.20 full-parity goal targets **DART 6.21**, per the maintainer's
release-closeout decision. This task retains reusable decisions,
verification recipes, and the [acceptance gates](#acceptance) for the
remaining work; [PLAN-622](../../plans/dashboard.md#plan-622-dart-6-deformable-body-feature-and-performance)
tracks its status.

DART 6 carries the Jain/Liu point-mass surface-flesh model implemented by
`SoftBodyNode`. Volumetric FEM remains out of scope. Representative soft-body
and soft-foot slices shipped through #3382, #3408, and #3423; they do not
establish full paper parity or robust push-recovery superiority.

- [Decisions](decisions.md): scope and durable technical owners.
- [Verification](verification.md): reusable commands and evidence boundaries.
- [Design](../../design/dart6_deformable_body.md): compatibility, activation,
  mass/COM, extension seam, data-layout, and detector contracts.
- [Paper targets](../../background/deformable_body_paper_targets.md): normalized
  research targets.

Preserve DART 6 public APIs, class layouts, vtables, default behavior, and
Gazebo compatibility. Work remains CPU-first. New GUI examples use `dart-demos`.
Promote any new durable findings before deleting this folder at completion.

## Acceptance

Rows close only with evidence; deferral is not an outcome. The
[parity matrix](02-paper-parity-matrix.md) holds each Jain/Liu row's paper
target, current evidence, and acceptance gate, plus the acceptance rule every
row must meet. The [execution plan](10-full-parity-execution-plan.md) holds
the Done rule (at least two clean independent reviews and a durable demo
artifact per behavior-bearing row), the performance-parity semantics, and what
each remaining row still needs.

Beyond the rows, apply the competitive envelope to the performance evidence
and complete the pre-default `dart` detector gates: coverage, allocation,
determinism, and same-host performance. Only two items accept an approved
disposition instead of evidence: multicore scaling (a negative disposition)
and a complete `pixi run bm-soft-body-paired` artifact.
