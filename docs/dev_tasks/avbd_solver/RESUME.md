# Resume: AVBD Solver

## Last Session Summary

The AVBD paper, project page, 2D demo source, and 3D demo source were inspected
and folded into PLAN-104. The implementation now has an internal scalar row
utility, deterministic row-key/inventory warm-start cache, active half-space
contact-normal block stamping, a standalone mass-spring AVBD contact-normal
driver, scalar hard point-attachment row stamping, a standalone mass-spring AVBD
attachment driver, scalar finite-stiffness spring rows, and narrow opt-in World
VBD paths for active static half-space contact-normal rows, pinned/scripted hard
attachment rows, and progressive finite-stiffness spring rows in supported
serial mass-spring scenes. Supported contact-normal, friction-tangent,
attachment, and finite-stiffness spring requests now run together through one
combined serial mass-spring AVBD row solve. The implementation also wires
finite-stiffness tetrahedral material rows into supported serial, frictionless
pure-tet World scenes, using a dimensionless material scale, strain-norm row
error, separate tet-row diagnostics, and coexistence with the existing lagged
VBD self-contact penalty. Contact-free finite-stiffness mass-spring scenes with
an inert material friction coefficient now stay on the AVBD row path instead of
falling back when there is no active contact or self-contact friction source. A
bounded friction-tangent row primitive now
participates in the serial mass-spring AVBD row driver, and supported
static-contact mass-spring World scenes generate two tangent rows per active
contact-normal row. Static box obstacle contact rows now include face/edge/corner
feature IDs so normal and friction rows reset when the contacted box manifold
changes while still warm-starting same-feature penetrations, and persisting
static half-space friction rows project their decayed dual into the current
tangent basis when smooth obstacle normals change. Persisting self-contact
friction rows project their generalized tangential dual into the current 12D
tangent stencil. Adjacent tangent-row pairs now use the lagged tangential dual to
switch between static sticking and dynamic sliding and project the paired force
to the circular Coulomb cone.
Supported serial mass-spring
self-contact scenes can also generate one AVBD self-contact normal row per
lagged point-triangle / edge-edge primitive, stamp each incident local vertex
through the shared scalar row, and report `vbdAvbdSelfContactNormalRows`.
Self-contact friction tangent rows now reuse lagged point-triangle / edge-edge
tangent stencils in the combined mass-spring row driver, with supported World
generation for serial mass-spring self-contact scenes and focused coverage for
tangent direction, static/dynamic pair switching, circular-cone projection,
tangential-motion resistance, World row generation, and World-level tangential
slip reduction. Static-contact and self-contact friction rows also have a
combined-row coexistence regression so both row families can participate in the
same supported AVBD mass-spring solve. Pure tetrahedral finite-stiffness scenes
can now also combine AVBD self-contact normal rows and matching self-contact
friction tangent rows in the same serial tet solve when
`useAvbdSelfContactNormalRows` is requested, with row-counter coverage for tet
finite-stiffness, self-contact normal, and self-contact friction families.
Unsupported mixed spring-plus-tet, mass-spring self-contact without the
self-contact AVBD flag, Chebyshev, Rayleigh-damped, parallel, and
unsupported-row requests have explicit fallback coverage that keeps them on the
existing VBD path. The first internal rigid contact-stage AVBD activation is
now guarded by `RigidAvbdContactConfig`: supported free rigid-body contacts use
the private World-contact snapshot and 6-DOF row solve as a velocity projection
consumed by the standard rigid position stage, while unsupported envelopes fall
back to sequential impulses. The private rigid contact snapshot also derives box
face/edge/corner endpoint feature IDs plus cylinder side/cap/rim and capsule
side/top-cap/bottom-cap endpoint feature IDs and scopes row ordinals per
canonical endpoint pair so unrelated manifolds do not perturb row identity. The
private rigid block path also has
point-joint linear, angular, and combined row builders for fixed-anchor
translation and orientation constraints, with step-start previous constraint
values seeded for AVBD alpha regularization. Those private point-joint rows can
now be appended to the World rigid snapshot/solve/apply wrapper and combined
step helper from world-space point-joint inputs; a private fixed-joint ECS
extractor plus step-helper overload now covers rigid-body-linked joint
entities, and the internal contact-stage AVBD opt-in can now project those
fixed-joint rows with or without active contacts. A private helper now derives
the AVBD fixed-joint config from the current rigid-body pair pose, preserving
the child-origin anchor and relative orientation for rigid-body-linked fixed
joints while rejecting multibody link endpoints. Public multibody joint
extraction is still not wired. The current local slice adds private per-axis
linear and angular masks to those point-joint builders, preserving all-axis
fixed-joint behavior while letting later hinge/revolute and limited-DOF configs
reuse the same descriptors, row inventory, and alpha-regularized warm starts.
It also adds named private revolute and prismatic point-joint builders that
construct arbitrary joint-axis bases, leave the hinge or translation axis free,
and preserve configured axes/masks through World point-joint input, snapshot
assembly, and solve coverage. Simulation-entry current-pose initialization and
extraction now also cover private rigid-body ECS revolute/prismatic joint
entities, deriving the same axes/masks from their configured joint axis while
keeping multibody link endpoints unsupported until articulated AVBD state
exists. Public experimental `World` facades now expose free rigid-body
revolute and prismatic joints through C++ and dartpy, with generated stubs,
focused C++/Python tests, and the categorized `sx_rigid_limited_joints`
py-demo. This branch also adds the first AVBD-specific `py-demos`
rigid-constraint scene, `avbd_rigid_fixed_joint_contact`, so users can inspect
the public fixed-joint row path while ordinary rigid contact acts on the
payload. The current follow-up adds private angular-motor rows and rigid
fracture dual-threshold/reset helpers in the 6-DOF row kernel, then wires
public rigid-body revolute velocity actuators into those bounded AVBD motor
rows with persistent contact-stage motor inventory, focused C++ coverage, and
the categorized `avbd_rigid_revolute_motor` py-demo. The AVBD dashboard
benchmark target also has a first end-to-end public-World
`BM_AvbdRigidRevoluteMotorStep` row for that motor path. This is still a narrow
free-rigid-body path only: no public fracture lifecycle, articulated multibody
state path, GPU path, paper-corpus demo, broad motor lifecycle coverage, or
paper/reference benchmark packet is complete.

## Current Branch

`feature/avbd-rigid-motor-fracture-rows` - checkpoint branch based on
current `origin/main`, including the scalar-row foundation, mass-spring AVBD row
families, standalone tet-material rows, and World wiring for supported pure-tet
finite-stiffness material rows, plus supported World static-contact friction
tangent rows, static box feature IDs, static half-space tangent dual projection,
self-contact tangent dual projection, supported World self-contact
normal/friction rows, combined static/self-contact friction row coexistence
coverage, and the first private 6-DOF rigid block plus contact/friction
point-pair row foundation. Private rigid point-pair friction tangent pairs now
reuse the deformable friction semantics for lagged-dual static/dynamic
switching and circular Coulomb-cone projection, and a private serial rigid row
driver now sweeps point attachments, contact-normal point pairs, and paired
friction tangent rows. A private rigid contact-manifold row builder now turns
active contact points with stable endpoint feature IDs into warm-started normal
and paired tangent rows for that driver. A private World-contact snapshot helper
now translates rigid-body `World::collide()` contacts into those
manifold-point inputs and verifies that they can drive the private serial rigid
row solve plus dynamic rigid-body ECS writeback through a combined private
wrapper. The current branch additionally adds the first internal
`RigidAvbdContactConfig` contact-stage activation for supported free rigid-body
contacts as a velocity-level projection and box-feature/pair-scoped rigid
contact row identity for the private snapshot path, plus private point-joint
linear, angular, and combined row builders with World snapshot/step
append/solve/apply coverage for world-space point-joint inputs and a private
fixed-joint ECS extractor plus step-helper overload for rigid-body-linked joint
entities, plus internal contact-stage velocity-projection coverage for those
fixed-joint rows with or without active contacts. This checkpoint adds a private
fixed-joint pose bridge that converts a rigid-body-linked fixed joint's current
pose into AVBD local anchors and target relative orientation, initializes
missing private AVBD fixed-joint configs for opt-in rigid bodies at simulation
entry, and adds regression coverage that multibody links are not silently
treated as rigid AVBD bodies. It also adds masked private point-joint row
generation for constrained linear/angular axes, plus named private
revolute/prismatic point-joint configs with arbitrary joint-axis bases and
private rigid-body ECS current-pose extraction for those one-DOF joint
entities. The current checkpoint adds public free rigid-body revolute/prismatic
facades, dartpy bindings/stubs, focused C++/Python coverage, and the
`sx_rigid_limited_joints` py-demo on top of that private path. This branch also
adds the `avbd_rigid_fixed_joint_contact` Python demo scene and catalog/test/doc
coverage for the first narrow user-visible AVBD rigid-constraint showcase. The
current local follow-up adds private angular-motor row construction, bounded
motor descriptors, solver-direction coverage, rigid fracture
dual-threshold/reset helpers, public rigid-body revolute velocity actuator
extraction into AVBD motor rows, persistent motor inventory in the contact
stage, the `avbd_rigid_revolute_motor` py-demo, and the AVBD dashboard
`BM_AvbdRigidRevoluteMotorStep` benchmark row.

## Immediate Next Step

Continue the next bounded AVBD contact/friction or rigid-block step:
articulated World wiring, full narrow-phase feature extraction, public fracture
lifecycle, or broader motor benchmarks are the preferred row-family gaps now that
private dynamic/rigid contact feature IDs, canonical two-endpoint row keys, and
normal/friction row descriptor helpers plus private rigid contact/friction
point-pair constructors, paired friction-cone helpers, and a private serial
rigid row driver plus private rigid contact-manifold row builder and
World-contact snapshot/solve/writeback helpers plus a combined private wrapper,
first internal contact-stage activation, box-feature/pair-scoped row identity,
private cylinder side/cap/rim and capsule side/top-cap/bottom-cap endpoint
features, and private point-joint linear/angular/combined rows with step-start
previous constraint values and private World snapshot/step append/solve/apply
coverage plus fixed-joint ECS extraction through the step helper and an
explicit current-pose rigid-body fixed-joint config bridge plus simulation-entry
config initialization for opt-in rigid bodies, masked private point-joint row
generation for constrained linear/angular axes, and named private
revolute/prismatic point-joint configs with arbitrary joint-axis bases plus
private rigid-body ECS current-pose extraction for those one-DOF joint
entities, and public free rigid-body revolute/prismatic facades with dartpy
bindings, stubs, tests, and py-demo coverage, plus fixed-joint/contact and
revolute-motor AVBD py-demos exist and private fracture helpers are starting.
Keep the supported envelope narrow and preserve fallback coverage for topology
mixes,
damping/acceleration, parallel solves, and unsupported requested row
combinations.

## Context That Would Be Lost

- AVBD belongs under PLAN-104 because it is the hard-constraint continuation of
  the VBD solver family, not a duplicate roadmap entry.
- Completion must preserve the full paper bar: all algorithms/features, CPU and
  GPU implementations, all paper/site/video/demo scenes, and benchmark JSON
  proving DART beats reference and paper numbers.
- The 2D reference demo source has 19 scenes; the 3D source has 14 scenes. Both
  are clarity references, not optimized baselines, but DART still must beat them
  before making CPU performance claims.
- The current AVBD code intentionally covers scalar row equations, row
  inventory warm starting, one CPU contact-normal kernel, one CPU hard
  attachment kernel, one CPU finite-stiffness spring kernel, and narrow
  internal World opt-ins for static contact-normal rows, hard point-attachment
  rows, and finite-stiffness spring rows, including a combined serial
  mass-spring row solve for those three families. Pure-tet World scenes can now
  use the tet-material finite-stiffness row path and, when requested, combine it
  with AVBD self-contact normal/friction rows. Supported static-contact
  mass-spring World scenes generate bounded friction-tangent rows with pairwise
  static/dynamic switching, and static box obstacle row keys now distinguish
  faces, edges, and corners. Persisting static half-space friction rows project
  their decayed tangential dual across changing smooth tangent bases, and
  persisting self-contact friction rows project their generalized tangential dual
  across changing 12D tangent stencils. Supported serial mass-spring
  self-contact scenes generate AVBD self-contact normal rows and matching
  self-contact friction rows when material friction is positive. A combined
  static/self-contact friction regression now guards both row families in one
  supported World solve. The first private rigid foundation now has a 6-DOF
  block accumulator, world-frame quaternion tangent update, inertia term, block
  solve, scalar rigid point-attachment row, and two-body point-pair row
  stamping. Point-pair rows also carry a scalar offset with private
  rigid-contact normal and bounded friction tangent constructors plus paired
  tangent static/dynamic switching and circular Coulomb-cone projection for
  future contact-manifold generation, and a private serial rigid row driver can
  sweep point attachments, contact-normal point pairs, and paired friction
  tangent rows. A private rigid contact-manifold row builder now converts active
  contact points with stable endpoint feature IDs into warm-started normal and
  paired tangent rows for that driver. A private World-contact snapshot helper
  now converts rigid-body `World::collide()` output into manifold-point inputs
  for that builder, runs them through the private serial rigid row solve, and
  writes dynamic rigid-body state back to the ECS through a combined private
  wrapper in focused tests. The first internal contact-stage activation now
  predicts rigid inertial targets from current velocities, solves supported
  opted-in contacts through the private 6-DOF rows, and projects the solved
  displacement back into rigid velocity. This does not imply
  hard-contact/friction completeness, full contact-manifold friction
  persistence, broad dynamic/rigid feature persistence, rigid/articulated
  joints, rigid/soft coupling, or GPU parity. Private dynamic/rigid contact
  identity helpers now pack contact feature kind/index IDs, canonicalize
  two-endpoint contact row keys, derive private box face/edge/corner endpoint
  feature IDs and cylinder side/cap/rim plus capsule side/top-cap/bottom-cap
  endpoint features for rigid snapshots, scope row ordinals per canonical
  endpoint pair, and create normal/friction/joint-linear/joint-angular row
  descriptors, with private point-joint rows now seeding step-start previous
  constraint values and participating in the private World snapshot/solve/apply
  wrapper and combined step helper from world-space point-joint inputs, plus
  private fixed-joint ECS extraction through the step helper and internal
  contact-stage velocity projection for rigid-body-linked joint entities with
  or without active contacts. Named private revolute and prismatic point-joint
  configs now preserve arbitrary joint-axis bases and leave one rotational or
  translational axis free, and private rigid-body ECS revolute/prismatic joint
  entities now initialize and extract through the same current-pose bridge.
  Public rigid-body revolute velocity actuators now extract to AVBD motor rows,
  but full narrow-phase feature extraction, public fracture lifecycle, broader
  motor lifecycle/benchmark coverage, and public articulated World joint wiring
  are not solved yet.

## How to Resume

```bash
git status --short --branch
pixi run build-simulation-experimental-tests
build/default/cpp/Release/bin/test_avbd_constraint
build/default/cpp/Release/bin/test_avbd_rigid_block
build/default/cpp/Release/bin/test_boxed_lcp_contact --gtest_filter='AvbdContact.*:BoxedLcpContact.MethodSelectorReflectsConstruction'
build/default/cpp/Release/bin/test_vbd_combined_descent --gtest_filter='VbdCombinedDescent.AvbdSelfContact*'
build/default/cpp/Release/bin/test_vbd_attachment
build/default/cpp/Release/bin/test_vbd_finite_stiffness
build/default/cpp/Release/bin/test_vbd_contact --gtest_filter='VbdContact.Avbd*'
build/default/cpp/Release/bin/test_vbd_world_solver --gtest_filter='VbdWorldSolver.AvbdTetRowsCombineSelfContactFrictionRows:VbdWorldSolver.AvbdContactAndSelfContactFrictionRowsCombine:VbdWorldSolver.AvbdSelfContactFrictionRowsReduceTangentialMotion:VbdWorldSolver.AvbdSelfContactNormalRowsIncludeFrictionTangentRows:VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial:VbdWorldSolver.AvbdFiniteStiffnessRowsFallbackForUnsupportedEnvelopes:VbdWorldSolver.AvbdContactNormalRowsFallbackForFriction:VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody:VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness:VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode:VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact'
```

Then continue Phase A1 from
[`README.md#immediate-next-steps`](README.md#immediate-next-steps).
