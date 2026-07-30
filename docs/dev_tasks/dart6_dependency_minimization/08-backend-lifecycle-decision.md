# Collision backend lifecycle decision

> Refreshed 2026-07-30 after PR #3381 merged. This task document owns the
> later-release sequencing decision. Durable current-state architecture lives
> in
> [DART 6 Collision Backends](../../design/dart6_collision_backends.md).

## Current DART 6.20 state

PR #3381 established the DART-owned collision backend as:

- public class `dart::collision::DARTCollisionDetector`;
- factory key `"dart"`;
- source home `dart/collision/dart/`; and
- the implementation behind the released `DARTCollide` entry points.

FCL remains the built-in default and a core dependency. FCL, Bullet, and ODE
remain real backend implementations with their existing factory keys,
installed components, headers, and package behavior.

This is the complete DART 6.20 boundary. It does not authorize a default flip,
backend facade conversion, component removal, or dependency removal on
`release-6.20`.

## Goal

After a future release proves and accepts the DART detector as its default,
reduce the external collision dependency footprint while preserving DART 6
source compatibility and the downstream gz-physics/gz-sim contract.

The sequence is architectural. It is not tied to a future branch number until
that branch and its milestone exist and a maintainer explicitly authorizes the
work.

## Decision 1 — canonical DART backend

The DART-owned backend is `DARTCollisionDetector`, selected by `"dart"`.
`CollisionDetectorType::Dart` and the dartpy `DARTCollisionDetector` binding
refer to the same backend.

The complete default-selection surface remains:

- both `ConstraintSolver` constructors;
- `WorldConfig`;
- `World` detector resolution; and
- parser fallback behavior.

Any later default change must update and verify that whole surface together.

## Decision 2 — ordered external-backend lifecycle

External backends may become compatibility facades only after the following
gates land in order:

1. **Default-flip release**
   - Refresh the correctness, determinism, performance, ABI, package, visual,
     and downstream baselines on the proposing release.
   - Prove `DARTCollisionDetector` meets the accepted bar without contact loss,
     changed sleeping behavior, cap hits, non-finite state, or hidden physics
     changes.
   - Change the entire default-selection surface in one reviewed change.
2. **Deprecation release**
   - Keep FCL, Bullet, and ODE implementations and components functional.
   - Add actionable migration messages to creation paths or constructors
     without breaking subclassing or warning-clean downstream builds.
   - Publish changelog and migration guidance.
3. **Dependency-decoupling release**
   - Remove FCL types from core-installed headers before making FCL optional.
   - Rebuild FCL, Bullet, and ODE components as approved compatibility facades.
   - Preserve public classes, component names, and factory keys unless a
     separately coordinated migration retires them.
   - Prove clean configuration and installed-prefix consumption with each
     external package disabled.

`DART_DEPRECATED(version)` carries no message. A future deprecation change
therefore needs message-bearing attributes or a dedicated macro, plus direct
`-Werror` validation in gz-physics.

## Downstream compatibility matrix

| gz-physics requirement | Lifecycle obligation |
| --- | --- |
| Selects `"fcl"`, `"bullet"`, `"ode"`, and `"dart"` | Keep all four keys functional through any facade transition |
| Finds `collision-bullet` and `collision-ode` components | Preserve component discovery and linkability until a coordinated migration |
| Subclasses `OdeCollisionDetector` | Keep the ODE surface subclassable or first remove that inheritance downstream |
| Reads DART 6 `Contact` fields | Preserve point, normal, penetration depth, force, and collision-object semantics |
| Applies per-pair contact limits | Preserve `CollisionOption.maxNumContactsPerPair` behavior |
| Uses distance, raycast, and voxel paths | Test these capabilities directly in the downstream gate |

## DART 6.20 acceptance record

PR #3381's exact reviewed head `64d476b68a` passed:

- 155 C++ tests and 223 dartpy tests;
- 43 focused DART detector tests in Release and assertions-enabled builds;
- installed-header, component, ABI-layout, and `DARTCollide` symbol canaries;
- the pinned gz-physics and gz-sim integration gates;
- detector-specific text and visual evidence; and
- incumbent FCL, Bullet, and ODE no-regression comparisons.

The terminal Linux run was `30510684936`. Those results establish the DART
6.20 state; a future default or dependency decision must recapture evidence on
its own parent and candidate heads.

## Maintainer ratification point

Before an ODE facade is implemented, choose one path:

1. preserve a subclassable `OdeCollisionDetector` facade for
   `GzOdeCollisionDetector`; or
2. coordinate a gz-physics change that removes the inheritance first.

No future facade or dependency-removal implementation should begin until a
proposing release and milestone exist and a maintainer explicitly authorizes
the packet.
