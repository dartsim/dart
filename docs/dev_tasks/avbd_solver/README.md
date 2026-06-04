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
    half-space contact-normal rows in supported serial mass-spring static
    contact scenes. The scratch cache now carries stable row IDs from
    body/entity, vertex, and static obstacle feature IDs. Static box obstacle
    feature IDs now distinguish faces, edges, and corners so AVBD normal and
    friction rows reset across box-manifold changes while warm-starting across
    small same-feature penetrations. Persisting static half-space friction rows
    also project their decayed tangential dual into the current tangent basis
    when a smooth obstacle normal changes, and persisting self-contact friction
    rows project their generalized tangential dual into the current 12D tangent
    stencil, with regression coverage in
    `VbdContact.AvbdBoxContactFeatureCodeSeparatesBoxManifolds`,
    `VbdContact.AvbdFrictionDualProjectionPreservesWorldImpulse`, and
    `VbdCombinedDescent.AvbdSelfContactFrictionDualProjectionPreservesGeneralizedImpulse`,
    plus
    `VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact`.
  - Narrow World wiring also started behind internal
    `DeformableVbdConfig::useAvbdAttachmentRows` for pinned or scripted
    deformable nodes in supported serial mass-spring scenes. It keeps
    attachment rows in per-body scratch storage, uses rest-position targets for
    ordinary pinned nodes and boundary targets for active Dirichlet anchors,
    can combine with the supported static-contact row envelope, and exposes
    only internal per-step row counters for tests such as
    `VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode`.
  - First finite-stiffness deformable row slice started:
    `AvbdSpringFiniteStiffnessRow`,
    `updateAvbdSpringFiniteStiffnessRow`, and
    `blockDescentMassSpringAvbdFiniteStiffness`, with focused
    `VbdFiniteStiffness.*` tests for stretch sign, ramp/cap behavior,
    finite-row warm starting, and driver updates.
  - First finite-stiffness tetrahedral material row slice started:
    `AvbdTetMaterialFiniteStiffnessRow`,
    `avbdTetMaterialConstraintValue`, and
    `blockDescentTetMeshAvbdFiniteStiffness`, with focused
    `VbdFiniteStiffness.*` tests for strain-norm row error, unit-scale
    ramp/cap behavior, and standalone tet-material driver updates.
  - Narrow World wiring also started behind internal
    `DeformableVbdConfig::useAvbdFiniteStiffnessRows` for progressive spring
    stiffness rows in supported serial mass-spring scenes, with row-counter and
    behavior coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain`.
  - The same internal finite-stiffness flag now also wires progressive
    tetrahedral material rows for supported serial, frictionless pure-tet
    scenes, with a dimensionless Lamé multiplier, a separate
    `vbdAvbdFiniteStiffnessTetRows` diagnostic counter, coexistence with the
    existing lagged VBD self-contact penalty, and regression coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial`.
    Contact-free finite-stiffness mass-spring scenes now stay on the AVBD row
    path when the material has a friction coefficient but no active contact or
    self-contact friction source, with coverage in
    `VbdWorldSolver.AvbdFiniteStiffnessRowsIgnoreUnusedFrictionCoefficient`.
    Pure-tet finite-stiffness scenes can now also combine AVBD self-contact
    normal rows and bounded self-contact friction tangent rows in the same
    serial tet solve when `useAvbdSelfContactNormalRows` is requested, with
    coverage in
    `VbdWorldSolver.AvbdTetRowsCombineSelfContactFrictionRows`.
  - The supported World mass-spring envelope now routes contact-normal,
    attachment, and finite-stiffness spring rows through one combined serial
    AVBD row solve, with regression coverage in
    `VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness`.
  - First bounded friction-tangent row slice started:
    `AvbdHalfSpaceFrictionRow`, `avbdFrictionTangentBounds`,
    `addAvbdHalfSpaceFrictionTangent`, and
    `updateAvbdHalfSpaceFrictionTangentRow`, with optional participation in
    `blockDescentMassSpringAvbdRows` and focused `VbdContact.Avbd*` coverage
    for Coulomb clamping, dual/stiffness updates, and tangential-motion
    resistance. Supported static-contact mass-spring World scenes now generate
    two bounded tangent rows per active AVBD contact normal, with
    `VbdWorldSolver.AvbdContactNormalRowsIncludeFrictionTangentRows` and
    `VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody` coverage.
    Adjacent tangent-row pairs now use the lagged tangential dual to switch
    between static sticking and dynamic sliding, and project the paired force to
    the circular Coulomb cone with focused
    `VbdContact.AvbdFrictionTangentPair*` coverage.
  - First self-contact normal row World slice started:
    `AvbdSelfContactNormalRow`, `avbdSelfContactNormalConstraintValue`,
    `addAvbdSelfContactNormal`, and `updateAvbdSelfContactNormalRow`. The
    kernel uses the IPC point-triangle / edge-edge barrier only to recover the
    local repulsion direction, then stamps an AVBD hard normal row with focused
    `VbdCombinedDescent.AvbdSelfContact*` coverage for direction, dual/stiffness
    updates, edge-edge rows, and inactive-band no-ops. Supported serial
    mass-spring self-contact scenes can opt in through
    `DeformableVbdConfig::useAvbdSelfContactNormalRows`; World generation keeps
    one scalar row per point-triangle / edge-edge primitive, stamps each
    incident local vertex through the lagged adjacency, reports
    `vbdAvbdSelfContactNormalRows`, and has regression coverage in
    `VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart`.
  - First self-contact friction row slice wired:
    `AvbdSelfContactFrictionRow`,
    `addAvbdSelfContactFrictionTangent`, and
    `updateAvbdSelfContactFrictionTangentRow`. The row reuses the IPC
    point-triangle / edge-edge tangent stencils against lagged primitive
    positions, participates in `blockDescentMassSpringAvbdRows`, has supported
    World generation for serial mass-spring self-contact scenes, and has
    focused `VbdCombinedDescent.AvbdSelfContactFriction*` plus
    `VbdWorldSolver.AvbdSelfContactNormalRowsIncludeFrictionTangentRows`
    and `VbdWorldSolver.AvbdSelfContactFrictionRowsReduceTangentialMotion`
    coverage for tangent direction, static/dynamic pair switching,
    circular-cone projection, tangential-motion resistance, World row
    generation, and World-level tangential slip reduction. Static-contact and
    self-contact friction rows also have combined-row coexistence coverage in
    `VbdWorldSolver.AvbdContactAndSelfContactFrictionRowsCombine`.
  - Fallback coverage now guards the unsupported World envelopes:
    mixed spring-plus-tet topology, mass-spring self-contact without the
    self-contact AVBD flag, Chebyshev, Rayleigh damping, parallel settings, and
    unsupported requested row families all keep using the existing VBD path
    without reporting partial AVBD row counters.
  - Still missing full contact-manifold friction persistence beyond current
    static half-space and lagged self-contact rows. The first private
    dynamic/rigid contact identity helpers now pack feature kind/index values,
    canonicalize two-endpoint row keys, and create private normal/friction row
    descriptors. The first private rigid contact-manifold row builder can turn
    active contact points into warm-started rigid normal/friction rows. The
    first private World-contact snapshot helper can translate rigid-body
    `World::collide()` contacts into those manifold points, run them through
    the private serial rigid row solve, write dynamic rigid-body state back to
    the ECS, and exercise the combined private build/solve/writeback wrapper in
    focused tests, but full row-family generation, articulated blocks, parallel
    dual/stiffness updates, and GPU parity remain missing.
- [ ] Phase A3: CPU 6-DOF rigid/articulated AVBD blocks.
  - First internal 6-DOF rigid block foundation started:
    `AvbdRigidBodyBlock`, quaternion tangent-step helpers,
    `addAvbdRigidBodyInertiaTerm`, `solveAvbdRigidBodyBlock`,
    `applyAvbdRigidBodyStep`, scalar rigid point-attachment rows, and private
    two-body rigid point-pair row stamping in
    `dart/simulation/experimental/detail/deformable_vbd/rigid_block_kernel.hpp`,
    with focused `AvbdRigidBlock.*` coverage. Point-pair rows now also carry a
    scalar offset, private constructors for rigid contact-normal and bounded
    contact-friction tangent rows, a paired tangent helper that switches
    between static sticking and dynamic sliding while projecting the force to a
    circular Coulomb cone, and a private serial row driver for point-attachment,
    contact-normal point-pair, and paired friction tangent rows. Future
    contact-manifold generation can reuse the same 6-DOF stamping path. The
    first private rigid contact-manifold builder now converts active contact
    points with stable endpoint feature IDs into warm-started normal rows and
    paired tangent rows for the private serial rigid row driver. The first
    private World-contact snapshot helper now extracts rigid contacts from
    `World::collide()` into that row-builder input and can drive the private
    serial rigid row solve plus dynamic rigid-body ECS writeback through a
    combined private wrapper. The first contact-stage AVBD activation now uses
    the internal `RigidAvbdContactConfig` for supported free rigid-body
    contacts: the stage predicts inertial targets from current velocities,
    solves the private 6-DOF rigid rows, and projects the solved displacement
    back into the velocity consumed by the standard position stage. The private
    rigid contact snapshot now derives box face/edge/corner, cylinder
    side/cap/rim, and capsule side/top-cap/bottom-cap endpoint feature IDs and
    scopes row ordinals per canonical endpoint pair for narrower warm-start
    persistence. Private point-joint linear, angular, and combined row builders
    now create the first
    rigid joint-row families for fixed-anchor translation and orientation
    constraints through the existing rigid row driver, seed their step-start
    constraint values for AVBD alpha regularization, and can be appended to the
    private World rigid snapshot/solve/apply wrapper and combined step helper
    from world-space point-joint inputs, including a private fixed-joint ECS
    extractor and step-helper overload for rigid-body-linked joint entities.
    The internal contact-stage AVBD opt-in can also project those fixed-joint
    rows with or without active contact rows. A private current-pose bridge can
    now derive AVBD local anchors and target relative orientation for
    rigid-body-linked fixed joints. Missing private AVBD fixed-joint configs are
    initialized from the simulation-entry pose only for opt-in rigid bodies, and
    multibody link endpoints are explicitly rejected until those bodies have a
    real articulated AVBD state path. The private point-joint builders now also
    accept per-axis linear and angular masks, preserving the all-axis
    fixed-joint behavior while letting future hinge/revolute and limited-DOF row
    configs reuse the same descriptors and warm-start inventory. The same
    private row path now has named revolute and prismatic point-joint builders
    that construct arbitrary joint-axis bases, leave the hinge or translation
    axis free, and preserve the configured axes/masks through World point-joint
    input, snapshot assembly, and solve coverage. Simulation-entry
    current-pose initialization and extraction now also cover private
    rigid-body ECS revolute/prismatic joint entities, deriving the same
    axes/masks from their configured joint axis while preserving explicit
    multibody rejection until articulated AVBD state exists.
    Public multibody joint extraction is still not wired.
    Unsupported envelopes still fall back to sequential impulses. This is not
    full narrow-phase feature extraction, not full rigid contact/joint rows, and
    not articulated joint support yet.
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

1. Continue the next bounded AVBD contact/friction or rigid-block slice:
   true rigid/articulated World wiring, full narrow-phase feature extraction,
   public rigid-body limited-DOF facades with py-demo coverage, or the next
   motor/fracture row family are the preferred next gaps now that static box
   feature IDs,
   private dynamic/rigid contact feature IDs and descriptor
   helpers, static half-space tangent dual projection, self-contact
   tangent dual projection, static contact/friction, attachments,
   finite-stiffness rows, self-contact normals, pairwise static/dynamic friction
   switching, supported World self-contact friction rows, pure-tet self-contact
   friction rows, combined static/self-contact friction row coexistence, and the
   first private 6-DOF rigid block plus contact/friction point-pair row
   foundation, rigid point-pair friction-cone helper, private serial rigid row
   driver, private rigid contact-manifold row builder, private World-contact
   snapshot/solve/writeback helpers, combined private wrapper, first internal
   `RigidAvbdContactConfig` contact-stage velocity-projection activation,
   box-feature/pair-scoped rigid contact row identity, cylinder side/cap/rim
   and capsule side/top-cap/bottom-cap endpoint features, private rigid
   point-joint linear/angular/combined row builders with step-start previous
   constraint values, and private World
   snapshot/step point-joint append/solve/apply coverage plus fixed-joint ECS
   extraction through the step helper and the internal contact-stage velocity
   projection with or without active contacts, plus an explicit rigid-body
   fixed-joint current-pose config bridge and simulation-entry config
   initialization for opt-in rigid bodies, masked private point-joint row
   generation for constrained linear/angular axes, and private
   revolute/prismatic point-joint configs with arbitrary joint-axis bases plus
   private rigid-body ECS current-pose extraction for those one-DOF joint
   entities, have narrow CPU paths.
2. In parallel planning, keep full friction cones, rigid/articulated rows, GPU
   parity, demos, and benchmark packets as open AVBD parity gates rather than
   completion claims.
