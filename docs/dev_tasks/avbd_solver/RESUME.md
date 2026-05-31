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
serial, frictionless mass-spring scenes. Unsupported tetrahedral, frictional,
self-contact, Chebyshev, Rayleigh-damped, and parallel requests now have
explicit fallback coverage that keeps them on the existing VBD path. Supported
contact-normal, attachment, and finite-stiffness spring requests now run
together through one combined serial mass-spring AVBD row solve. The next slice
also started standalone finite-stiffness tetrahedral material rows with a
dimensionless material scale, strain-norm row error, and a tet-only block
descent driver/test, but this is not World-wired yet.

## Current Branch

`feature/avbd-plan104-foundation` - checkpoint commits based on current
`origin/main`, plus local uncommitted standalone finite-stiffness tetrahedral
material row work.

## Immediate Next Step

Wire finite-stiffness tetrahedral material rows into the supported World VBD
envelope, then add the next bounded row family, starting with contact/friction
bounds or self-contact rows in the same combined-row solve.

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
  mass-spring row solve for those three families. A standalone tet-material
  finite-stiffness row driver now exists, but it is not yet a scene-level AVBD
  solver and does not imply World tetrahedral row coverage, hard-contact/friction
  completeness, self-contact support, rigid/soft coupling, or GPU parity.

## How to Resume

```bash
git status --short --branch
pixi run build-simulation-experimental-tests
build/default/cpp/Release/bin/test_avbd_constraint
build/default/cpp/Release/bin/test_vbd_attachment
build/default/cpp/Release/bin/test_vbd_finite_stiffness
build/default/cpp/Release/bin/test_vbd_contact --gtest_filter='VbdContact.Avbd*'
build/default/cpp/Release/bin/test_vbd_world_solver --gtest_filter='VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain:VbdWorldSolver.AvbdFiniteStiffnessRowsFallbackForUnsupportedEnvelopes:VbdWorldSolver.AvbdContactNormalRowsFallbackForFriction:VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness:VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode:VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact'
```

Then continue Phase A1 from
[`README.md#immediate-next-steps`](README.md#immediate-next-steps).
