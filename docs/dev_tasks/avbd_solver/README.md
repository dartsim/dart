# AVBD Solver - Dev Task

Implementation tracking for PLAN-104's Augmented Vertex Block Descent work.
Plan: [`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md).
Paper audit:
[`../../plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](../../plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md).

## Current Status

- [x] Phase A0: scalar row foundation started
  - Internal row update utility:
    `dart/simulation/experimental/detail/deformable_vbd/avbd_constraint.hpp`.
  - Focused tests:
    `tests/unit/simulation/experimental/deformable_vbd/test_avbd_constraint.cpp`.
- [ ] Phase A1: row model and persistent inventory started, not complete
  - Deterministic scalar-row descriptors, keys, warm-start state cache, and
    role/axis separation:
    `dart/simulation/experimental/detail/deformable_vbd/avbd_row_inventory.hpp`.
  - Still missing production row generation and storage for the full contact,
    friction, joint, motor, fracture, and attachment corpus.
- [ ] Phase A2: CPU deformable AVBD through the existing VBD body path
  - First active contact-normal kernel slice started:
    `addAvbdHalfSpaceContactNormal`,
    `updateAvbdHalfSpaceContactNormalRow`, and
    `blockDescentMassSpringAvbdGround`.
  - First non-contact hard attachment row slice started:
    `AvbdPointAttachmentRow`, `addAvbdPointAttachment`,
    `updateAvbdPointAttachmentRow`, and
    `blockDescentMassSpringAvbdAttachments`, with focused
    `VbdAttachment.*` tests for force direction, alpha regularization,
    per-axis warm-start persistence, and dual/stiffness growth.
  - Narrow World wiring started behind internal
    `DeformableVbdConfig::useAvbdContactNormalRows` for active static
    half-space contact-normal rows in supported serial, frictionless
    mass-spring scenes. The scratch cache now carries stable row IDs from
    body/entity, vertex, and static obstacle feature IDs, with regression
    coverage in `VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact`.
  - Narrow World wiring also started behind internal
    `DeformableVbdConfig::useAvbdAttachmentRows` for pinned or scripted
    deformable nodes in supported serial, frictionless mass-spring scenes. It
    keeps attachment rows in per-body scratch storage, uses rest-position
    targets for ordinary pinned nodes and boundary targets for active
    Dirichlet anchors, and exposes only internal per-step row counters for
    tests such as `VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode`.
  - First finite-stiffness deformable row slice started:
    `AvbdSpringFiniteStiffnessRow`,
    `updateAvbdSpringFiniteStiffnessRow`, and
    `blockDescentMassSpringAvbdFiniteStiffness`, with focused
    `VbdFiniteStiffness.*` tests for stretch sign, ramp/cap behavior,
    finite-row warm starting, and driver updates.
  - Narrow World wiring also started behind internal
    `DeformableVbdConfig::useAvbdFiniteStiffnessRows` for progressive spring
    stiffness rows in supported serial, contact-free, frictionless mass-spring
    scenes, with row-counter and behavior coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain`.
  - The supported World mass-spring envelope now routes contact-normal,
    attachment, and finite-stiffness spring rows through one combined serial
    AVBD row solve, with regression coverage in
    `VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness`.
  - Fallback coverage now guards the unsupported World envelopes: tetrahedral
    bodies, friction, self-contact, Chebyshev, Rayleigh damping, parallel
    settings, and unsupported requested row families all keep using the
    existing VBD path without reporting partial AVBD row counters.
  - Still missing finite-stiffness tetrahedral rows, self-contact, friction,
    full row-family generation, parallel dual/stiffness updates, and GPU
    parity.
- [ ] Phase A3: CPU 6-DOF rigid/articulated AVBD blocks.
- [ ] Phase A4: contact/friction bounds, static/dynamic friction switching, and
      quasi-Newton Hessian approximation.
- [ ] Phase A5: joints, motors, fracture, and breakable constraints.
- [ ] Phase A6: unified soft/rigid AVBD coupling.
- [ ] Phase G: GPU parity for all row families, candidate generation, and
      benchmark scenes.
- [ ] Phase D/P: every paper/demo scene plus benchmark JSON proving DART beats
      reference and paper CPU/GPU numbers.

## Goal

Implement AVBD as the hard-constraint continuation of DART's VBD solver family:
all paper/reference algorithms and features, CPU and GPU parity, complete
paper/site/video/demo reproduction in DART tests/benchmarks/`py-demos`, and
performance that beats the reference demo repositories and published paper
numbers.

## Non-Goals For Early Phases

- Do not expose AVBD, row storage, solver registries, CUDA types, or ECS details
  as public API.
- Do not vendor or runtime-link the AVBD demo repositories.
- Do not claim AVBD parity from the scalar row utility, a CPU-only slice, or one
  demo scene.

## Key Decisions

- **Reuse PLAN-104:** AVBD extends the VBD solver family, so the durable owner is
  PLAN-104 plus its AVBD paper gap audit rather than a duplicate plan.
- **Row foundation first:** The scalar row update equations are shared by hard
  contact, joints, attachments, friction limits, motors, fracture, and
  finite-stiffness ramping, so they are the first tested implementation slice.
- **Clean DART 7/8 architecture:** AVBD work may refactor internal solver,
  pipeline, row-storage, compute, and demo surfaces when that produces a cleaner
  long-term design.

## Immediate Next Steps

1. Extend finite-stiffness row coverage from springs to tetrahedral/material
   rows, then broaden diagnostics as each new row family becomes World-wired.
2. Add the next bounded AVBD row family, starting with contact/friction bounds
   or self-contact rows in the same combined-row solve.
