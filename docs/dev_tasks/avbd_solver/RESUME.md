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
VBD self-contact penalty. A bounded friction-tangent row primitive now
participates in the serial mass-spring AVBD row driver, and supported
static-contact mass-spring World scenes generate two tangent rows per active
contact-normal row. Adjacent tangent-row pairs now use the lagged tangential
dual to switch between static sticking and dynamic sliding and project the
paired force to the circular Coulomb cone. Supported serial mass-spring
self-contact scenes can also generate one AVBD self-contact normal row per
lagged point-triangle / edge-edge primitive, stamp each incident local vertex
through the shared scalar row, and report `vbdAvbdSelfContactNormalRows`.
Standalone self-contact friction tangent rows now reuse lagged point-triangle /
edge-edge tangent stencils in the combined mass-spring row driver, with focused
coverage for tangent direction, static/dynamic pair switching, circular-cone
projection, and tangential-motion resistance.
Unsupported mixed spring-plus-tet, mass-spring self-contact without the
self-contact AVBD flag, finite-stiffness-only friction scenes, Chebyshev,
Rayleigh-damped, parallel, and unsupported-row requests have explicit fallback
coverage that keeps them on the existing VBD path.

## Current Branch

`feature/avbd-plan104-foundation` - checkpoint commits based on current
`origin/main`, including the scalar-row foundation, mass-spring AVBD row
families, standalone tet-material rows, and World wiring for supported pure-tet
finite-stiffness material rows, plus supported World static-contact friction
tangent rows and supported World self-contact normal rows.

## Immediate Next Step

Start the next bounded AVBD contact/friction slice: World generation for
self-contact friction rows or fuller contact-manifold friction persistence are
the preferred row-family gaps. Keep the supported envelope narrow and preserve
fallback coverage for topology mixes, damping/acceleration, parallel solves, and
unsupported requested row combinations.

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
  use the tet-material finite-stiffness row path, supported static-contact
  mass-spring World scenes generate bounded friction-tangent rows with pairwise
  static/dynamic switching, and supported serial mass-spring self-contact scenes
  generate AVBD self-contact normal rows. Standalone self-contact friction rows
  now exist in the combined mass-spring driver, but World generation for those
  rows is still missing. This does not imply hard-contact/friction completeness,
  full contact-manifold friction persistence, rigid/soft coupling, or GPU
  parity.

## How to Resume

```bash
git status --short --branch
pixi run build-simulation-experimental-tests
build/default/cpp/Release/bin/test_avbd_constraint
build/default/cpp/Release/bin/test_vbd_combined_descent --gtest_filter='VbdCombinedDescent.AvbdSelfContact*'
build/default/cpp/Release/bin/test_vbd_attachment
build/default/cpp/Release/bin/test_vbd_finite_stiffness
build/default/cpp/Release/bin/test_vbd_contact --gtest_filter='VbdContact.Avbd*'
build/default/cpp/Release/bin/test_vbd_world_solver --gtest_filter='VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial:VbdWorldSolver.AvbdFiniteStiffnessRowsFallbackForUnsupportedEnvelopes:VbdWorldSolver.AvbdContactNormalRowsFallbackForFriction:VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody:VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness:VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode:VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact'
```

Then continue Phase A1 from
[`README.md#immediate-next-steps`](README.md#immediate-next-steps).
