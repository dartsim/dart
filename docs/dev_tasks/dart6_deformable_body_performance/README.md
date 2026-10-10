# DART 6 deformable-body parity and performance

The unmet DART 6.20 full-parity goal targets **DART 6.21**, per the maintainer's
release-closeout decision. This task retains reusable decisions and
verification recipes; [PLAN-622](../../plans/dashboard.md#plan-622-dart-6-deformable-body-feature-and-performance)
owns remaining work and acceptance gates.

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
