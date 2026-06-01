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
existing VBD path.

## Current Branch

`feature/avbd-plan104-rigid-block-foundation` - checkpoint commits based on current
`origin/main`, including the scalar-row foundation, mass-spring AVBD row
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
wrapper.

## Immediate Next Step

Continue the next bounded AVBD contact/friction or rigid-block slice:
contact-stage rigid AVBD activation, rigid contact/joint rows, or
rigid/articulated World wiring are the preferred row-family gaps now that
private dynamic/rigid contact feature IDs, canonical two-endpoint row keys, and
normal/friction row descriptor helpers plus private rigid contact/friction
point-pair constructors, paired friction-cone helpers, and a private serial
rigid row driver plus private rigid contact-manifold row builder and
World-contact snapshot/solve/writeback helpers plus a combined private wrapper
exist. Keep the supported envelope narrow and preserve fallback coverage for
topology mixes, damping/acceleration, parallel solves, and unsupported
requested row combinations.

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
  wrapper in focused tests, but there is still no contact-stage activation. This
  does not imply hard-contact/friction completeness, full contact-manifold
  friction
  persistence, broad dynamic/rigid feature persistence, rigid/articulated
  joints, rigid/soft coupling, or GPU parity. Private
  dynamic/rigid contact identity helpers now pack contact feature kind/index
  IDs, canonicalize two-endpoint contact row keys, and create normal/friction
  row descriptors, but World dynamic/rigid contact manifolds are not solved yet.

## How to Resume

```bash
git status --short --branch
pixi run build-simulation-experimental-tests
build/default/cpp/Release/bin/test_avbd_constraint
build/default/cpp/Release/bin/test_avbd_rigid_block
build/default/cpp/Release/bin/test_vbd_combined_descent --gtest_filter='VbdCombinedDescent.AvbdSelfContact*'
build/default/cpp/Release/bin/test_vbd_attachment
build/default/cpp/Release/bin/test_vbd_finite_stiffness
build/default/cpp/Release/bin/test_vbd_contact --gtest_filter='VbdContact.Avbd*'
build/default/cpp/Release/bin/test_vbd_world_solver --gtest_filter='VbdWorldSolver.AvbdTetRowsCombineSelfContactFrictionRows:VbdWorldSolver.AvbdContactAndSelfContactFrictionRowsCombine:VbdWorldSolver.AvbdSelfContactFrictionRowsReduceTangentialMotion:VbdWorldSolver.AvbdSelfContactNormalRowsIncludeFrictionTangentRows:VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial:VbdWorldSolver.AvbdFiniteStiffnessRowsFallbackForUnsupportedEnvelopes:VbdWorldSolver.AvbdContactNormalRowsFallbackForFriction:VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody:VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness:VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode:VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact'
```

Then continue Phase A1 from
[`README.md#immediate-next-steps`](README.md#immediate-next-steps).
