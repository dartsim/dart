# Native Collision Default - Dev Task

## Current Status

- [x] Native collision core exists under `dart/collision/native/`.
- [x] DART adapter exists under `dart/collision/dart/`.
- [x] Legacy `"experimental"` factory key is retained as a compatibility alias.
- [x] `dart` is the first-choice default detector in core world, constraint,
      and SKEL-loading paths.
- [x] Native coverage is proven against the DART feature surface currently
      served by FCL, Bullet, and ODE, including the final `VoxelGridShape`
      parity gap found by audit.
- [x] gz-physics compatibility is proven without downstream patches.
- [x] Comparative benchmarks prove native is at least as fast as the best
      legacy backend for required workloads. Primitive, narrow-phase,
      supported distance, raycast, raycast-batch, mesh-heavy, and
      mixed-primitive scenario cases pass. The previous mixed-primitives dense
      1000 loss is now a win through cached broad-phase snapshot reuse:
      native is 1.06 ms CPU mean versus Bullet at 1.69 ms.
- [x] FCL, Bullet, and ODE are no longer required collision dependencies. A
      core `dart` build, focused native/default C++ tests, and `dartpy` now
      build with all three disabled.

## Goal

Make DART's built-in/native collision detector the default implementation and
the long-term replacement for FCL, Bullet, and ODE collision backends. The
replacement must be evidence driven: feature coverage, behavior compatibility,
gz-physics compatibility, and performance must be verified before dependency
removal.

## North Star

DART should not require FCL, Bullet, or ODE for collision detection. The native
detector should cover the DART collision feature surface and beat the best
legacy backend on representative collision, distance, raycast, broadphase, and
simulation workloads.

## Non-Goals For This Tracking Task

- Redesigning the constraint solver beyond native collision integration hooks.
- Removing public legacy class names before gz-physics and downstream ABI needs
  are resolved.
- Keeping reference backends in the default build after they stop being needed
  for benchmarks and compatibility validation.

## Key Decisions

- The public default detector key is `dart`; `experimental` remains a temporary
  factory alias only for old saved worlds, tests, and downstream code.
- `dart/collision/native/` owns backend-independent algorithms and data
  structures; `dart/collision/dart/` owns DART API adaptation.
- Reference backends are validation tools during migration, not permanent
  runtime dependencies.
- gz-physics compatibility is a release gate. If gz-physics subclasses or names
  legacy detector classes, DART keeps facade types until gz-physics has a
  compatible path.
- Performance claims require benchmark output checked into this task or linked
  from CI artifacts; no subjective performance sign-off.

## Immediate Next Steps

1. Use the evidence in this folder when drafting the PR description.
2. Remove this working folder in the completion PR after its evidence has been
   transferred to the PR description.

## Completion Rules

This folder is working documentation. When native collision reaches the
completion criteria, extract only durable design notes into
`docs/onboarding/architecture.md`, then delete this folder in the same PR.
