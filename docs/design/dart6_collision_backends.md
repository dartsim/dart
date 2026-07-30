# DART 6 Collision Backends

This document owns the durable architecture and compatibility boundaries for
collision backends on DART 6.20. Active sequencing and verification evidence
live in the dependency-minimization plan and task documents.

## Current architecture

| Surface | DART 6.20 contract |
| --- | --- |
| Built-in default | FCL `PRIMITIVE` |
| DART-owned detector | `dart::collision::DARTCollisionDetector` |
| DART-owned factory key | `"dart"` |
| DART-owned source home | `dart/collision/dart/` |
| External implementations | FCL, Bullet, and ODE remain real backends |
| Supported factory keys | `"dart"`, `"fcl"`, `"bullet"`, and `"ode"` |
| Installed collision components | Unchanged |
| External collision dependencies | FCL remains core-required; Bullet and ODE remain optional |
| Released `DARTCollide` API | Preserved through thin adapters |
| Detector/group/object layouts | Preserved |
| C++ and Python floor | C++17 and pybind11 |

PR #3381 established this architecture on DART 6.20. Its squash merge is
`46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.

## DART detector contract

The `"dart"` factory key selects the DART-owned detector. Its supported
surface includes:

- the DART 6 collision-group, collision-object, filter, option, and result
  interfaces;
- primitive, plane, convex, mesh, soft-body, ellipsoid, cone, capsule, and
  voxel-backed collision paths covered by the release test corpus;
- distance, raycast, continuous-collision, and persistent-manifold behavior
  covered by detector-specific tests; and
- the released `DARTCollide` entry points and symbols.

Correctness is defined by finite state, deterministic detector results,
preserved contact semantics, accepted scene tolerances, and downstream
compatibility. Performance gains may not come from lost contacts, changed
sleeping behavior, cap hits, or altered physics.

## Default-selection boundary

FCL remains the DART 6.20 default. The selection boundary includes both
`ConstraintSolver` constructors, `WorldConfig`, `World` detector resolution,
and parser fallbacks. A later default change must update and verify that whole
surface as one compatibility decision.

## Downstream contract

gz-physics and gz-sim constrain later backend work:

- `find_package(DART COMPONENTS collision-bullet collision-ode ...)` must
  remain valid unless a coordinated downstream migration changes that
  contract.
- Factory keys `"fcl"`, `"bullet"`, `"ode"`, and `"dart"` must continue to
  resolve.
- `GzOdeCollisionDetector` subclasses `OdeCollisionDetector` and overrides
  collision methods. Any ODE compatibility facade must remain subclassable,
  or gz-physics must first coordinate removal of that inheritance.
- DART 6 `Contact` point, normal, penetration depth, force, and
  collision-object semantics remain the downstream data contract.
- Per-pair contact caps, distance, raycast, voxel, and installed-component
  behavior require direct downstream tests rather than source-shape
  inference.

## Later backend lifecycle

A future release may move from real external backends to compatibility facades
over `DARTCollisionDetector`, but only in this order:

1. Prove and approve the DART detector as the default on the proposing release
   with current correctness, determinism, performance, package, ABI, visual,
   and gz evidence.
2. Provide a functional deprecation period with migration messages while the
   FCL, Bullet, and ODE backends and components remain real.
3. Decouple FCL from core and remove external packages only after installed
   headers no longer expose their types and all compatibility components pass
   without those packages present.
4. Preserve the public detector classes, component names, and factory keys as
   facades unless a separately coordinated compatibility change retires them.

Performance alone is insufficient evidence for any stage.

## Non-goals

- DART 6.20 does not flip the default detector.
- DART 6.20 does not deprecate or remove FCL, Bullet, or ODE.
- DART 7's EnTT/C++23 world layer is reference material, not a DART 6
  compatibility argument.

## Evidence owners

- Active plan and acceptance evidence:
  `docs/dev_tasks/dart6_dependency_minimization/`.
- Release roadmap state: `docs/plans/dashboard.md`.
- Collision performance methodology: `docs/onboarding/profiling.md`.
- Verification and downstream gates: `docs/ai/verification.md`.
