# DART 6 Collision Backend Consolidation

This document owns the durable architecture and compatibility decisions behind
the DART 6 DART-owned collision backend. Active sequencing and evidence live in
the dependency-minimization plan and task documents.

## Decision

DART has one DART-owned collision backend:

- public class: `dart::collision::DARTCollisionDetector`;
- factory key: `"dart"`;
- source home: `dart/collision/dart/`.

The temporary `"native"` key and `NativeCollisionDetector` class family were
introduced only on the unreleased DART 6.20 branch. They were removed during
consolidation rather than retained as aliases. The
`dart::collision::native` namespace remains an internal implementation detail
inside `dart/collision/dart/`; it is not a public detector identity.

PR #3381 implemented this decision on DART 6.20. Its squash merge is
`46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.

## DART 6.20 compatibility boundary

| Surface | DART 6.20 result |
| --- | --- |
| Built-in default | FCL `PRIMITIVE` |
| Explicit `"dart"` selection | Consolidated DART-owned engine |
| Explicit `"native"` selection | Unsupported |
| FCL/Bullet/ODE implementations | Real backends, unchanged by consolidation |
| FCL/Bullet/ODE factory keys | Continue to resolve to their real backends |
| Installed collision components | Unchanged |
| External collision dependencies | Unchanged; FCL remains core-required |
| Released `DARTCollide` API | Preserved through thin adapters |
| Detector/group/object layouts | Preserved |
| C++ and Python floor | C++17 and pybind11 |

The default-selection boundary includes both `ConstraintSolver` constructors,
`WorldConfig`, `World` detector resolution, and parser fallbacks. A later
default change must treat that surface as one compatibility decision.

## Why consolidation precedes dependency removal

DART 6.20 temporarily carried the released narrowphase-only `"dart"` detector
and a newer DART-owned engine under the pre-release `"native"` name. Keeping
both would create two public identities, duplicated adapters, and an
unnecessary deprecation cycle for a name that downstream users had not
received.

Consolidation establishes one long-term DART-owned engine while keeping the
released DART 6 default and external backends stable. It does not itself prove
that a default flip or dependency removal is appropriate.

## Downstream contract

gz-physics and gz-sim constrain later backend work:

- `find_package(DART COMPONENTS collision-bullet collision-ode ...)` must
  remain valid unless a coordinated downstream migration changes that
  contract.
- Factory keys `"fcl"`, `"bullet"`, `"ode"`, and `"dart"` must resolve.
- `GzOdeCollisionDetector` subclasses `OdeCollisionDetector` and overrides
  collision methods. Any ODE compatibility facade must remain subclassable,
  or gz-physics must first coordinate removal of that inheritance.
- DART 6 `Contact` point, normal, penetration depth, force, and collision-object
  semantics remain the downstream data contract.
- Per-pair contact caps, distance, raycast, voxel, and installed-component
  behavior require direct downstream tests rather than source-shape inference.

## Later backend lifecycle

A future release may move from real external backends to compatibility facades
over `DARTCollisionDetector`, but only in this order:

1. Prove and approve the consolidated DART detector as the default on the
   proposing release with current correctness, determinism, performance,
   package, ABI, visual, and gz evidence.
2. Provide a functional deprecation period with migration messages while the
   FCL/Bullet/ODE backends and components remain real.
3. Decouple FCL from core and remove the external packages only after installed
   headers no longer expose their types and all compatibility components pass
   without the packages present.
4. Preserve the public detector classes, component names, and factory keys as
   facades unless a separately coordinated compatibility change retires them.

Performance alone is insufficient evidence. A faster row is rejected when it
loses contacts, changes sleeping or finite-state behavior, hits caps, or exceeds
the accepted scene tolerance.

## Non-goals

- DART 6.20 does not flip the default detector.
- DART 6.20 does not deprecate or remove FCL, Bullet, or ODE.
- The internal `native` namespace does not imply a supported `"native"`
  factory key.
- DART 7's EnTT/C++23 world layer is reference material, not a DART 6
  compatibility argument.

## Evidence owners

- Active plan and acceptance evidence:
  `docs/dev_tasks/dart6_dependency_minimization/`.
- Release roadmap state: `docs/plans/dashboard.md`.
- Collision performance methodology: `docs/onboarding/profiling.md`.
- Verification and downstream gates: `docs/ai/verification.md`.
