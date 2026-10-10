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

Each remaining Jain/Liu row closes only with evidence; deferral is not an
outcome. A row needs a runnable DART 6 scene or benchmark; correctness or
stability evidence with deterministic state or metric thresholds;
`dart`-versus-FCL contact and performance evidence when it depends on
deformable collision; single-thread and host-capped multi-thread CPU rows; a
SIMD-off and SIMD-on result when it touches vectorizable kernels; and a
comparison against the paper number or an approved normalized target.

| Row | Row-specific gate |
| --- | --- |
| Push recovery with motor noise | The push comparison reproduces (#3408, #3423); add the motor-noise variant plus `dart`-versus-FCL and multi-thread evidence. Single-trajectory thresholds are not ensemble evidence (#3431). |
| Noisy-floor biped | Seeded floor of 5 x 5 cm tiles whose vertices move by uniform 0-2 cm vertical and horizontal offsets; rigid-versus-soft outcome, contact count, and CoP trace. |
| Soft-contact walking | A soft-foot SIMBICON gait that walks rather than stepping in place, at the paper's cadence of one LCP solve per eight 0.5 ms controller steps, with contact counts, solver-time share, and a deterministic controller outcome. |
| Four-link flexible foot | Add the four-link flexible-rigid foot to the existing soft-foot comparison: three-way, same controller and seeded perturbations, with stability and contact metrics. |
| Finger flick | Hand contact scene with adaptive DOF and contact counts, LCP and collision profiler rows, and a launch-direction or CoP correctness gate. |
| Arm fold | Upper-body self-contact scene with `dart`-versus-FCL contact parity and a solver-time breakdown. |
| Pinch grasp | Manipulation scene with contact-count stability and an object-state checksum. |

Beyond the rows, apply the competitive envelope (in-tree backends plus
normalized paper metrics) to the performance evidence, and complete the
pre-default `dart` detector gates: coverage, allocation, determinism, and
same-host performance. Only two items accept an approved disposition instead
of evidence: multicore scaling (a negative disposition) and a complete
`pixi run bm-soft-body-paired` artifact.
