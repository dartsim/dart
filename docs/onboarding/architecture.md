# DART Classic Core Architecture

> **Scope:** the classic DART 6 core. `main` still carries the classic
> `dart/dynamics`, `dart/constraint`, and `dart/optimizer` modules
> (`Skeleton`/`BodyNode`/`Joint`, `ConstraintSolver`) in a frozen state; the
> DART 6 `World` and its simulation API were removed from `main` and live only
> on the maintained `release-6.*` line.
> DART 7 is a different engine: its architecture, data flow, and compute graph
> are mapped on the published
> [architecture page](../readthedocs/architecture.md), with the rationale in
> [`design/`](../design/README.md). Read this page when you touch the classic
> modules or need to understand DART 6 behavior used as comparison evidence.

## Module Layers

The classic core is a layered library; each layer depends only on the layers
below it. The library-context view on the architecture page shows the same
modules next to the DART 7 core and the dartpy and dartsim surfaces.

| Layer      | Modules                                                                       | Handbook page                            |
| ---------- | ----------------------------------------------------------------------------- | ---------------------------------------- |
| Foundation | `dart/common`, `dart/math`, `dart/lcpsolver`, `dart/simd`                     | this page (below), [profiling.md]        |
| Collision  | `dart/collision` (native detector; fcl, bullet, ode compatibility facades)    | [`dart/collision/AGENTS.md`]             |
| Dynamics   | `dart/dynamics` (`Skeleton`, `BodyNode`, `Joint`, shapes)                     | [dynamics.md]                            |
| Constraint | `dart/constraint` (`ConstraintSolver`, contact, joint limits)                 | [constraints.md]                         |
| Simulation | DART 6 `World` stepping and recording (`release-6.*` only), `dart/io` loading | [io-parsing.md], [aspect-system.md]      |
| Surfaces   | `dart/gui`, `dartpy`, `dartsim`                                               | [gui-rendering.md], [python-bindings.md] |

[profiling.md]: profiling.md
[`dart/collision/AGENTS.md`]: ../../dart/collision/AGENTS.md
[dynamics.md]: dynamics.md
[constraints.md]: constraints.md
[io-parsing.md]: io-parsing.md
[aspect-system.md]: aspect-system.md
[gui-rendering.md]: gui-rendering.md
[python-bindings.md]: python-bindings.md

## Math Module (`dart/math/`)

`dart/math` owns the Lie-group and geometry primitives shared by both engines:
`SO3`/`SE3` operations, spatial vectors and inertia, configuration spaces,
geometry helpers such as `computeSupportPolygon`, and the constants and helper
functions used by the solvers. The LCP solvers used by the classic constraint
layer live in `dart/lcpsolver` (Dantzig and PGS variants). The typed batch
strategy for the Lie group API is described in
[`design/lie_group_batch.md`](../design/lie_group_batch.md); the theory behind
the derivations is in [`background/`](../background/README.md).

## Classic Simulation Loop

On `release-6.*`, the classic `World::step()` runs collision detection,
constraint solving, and integration in generalized coordinates:

1. Compute forward dynamics for every `Skeleton` with Featherstone's
   articulated-body algorithm (`dart/dynamics`), using the composite rigid body
   and recursive Newton-Euler algorithms where mass matrices and inverse
   dynamics are requested.
2. Detect collisions through the configured `CollisionDetector` and build
   contact constraints (`dart/collision`, `dart/constraint`).
3. Solve the constraint problem as a boxed LCP over constrained groups
   (`ConstraintSolver`, `dart/lcpsolver`), applying joint limits, servo and
   friction constraints alongside contacts.
4. Integrate velocities and positions with semi-implicit Euler and apply the
   resulting state to every `Skeleton`; record frames when recording is on.

The DART 7 `World` replaces this loop with an ordered stage schedule per solver
family; the step-flow view on the architecture page shows which slots run for
each family, and
[`design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
explains why the two engines differ.

## Design Patterns Worth Knowing

- **Aspect system**: runtime extension of `Skeleton`, `BodyNode`, and `Joint`
  through `Aspect`, `State`, and `Properties` composites; see
  [aspect-system.md](aspect-system.md).
- **Strategy objects**: collision detectors, LCP solvers, and constraint
  solvers are swappable implementations behind stable interfaces.
- **Lazy evaluation**: transforms, Jacobians, and mass matrices are cached and
  recomputed only when the dependent state changes.
- **Resource retrievers**: model loading resolves `package://` and `file://`
  URIs through composable retrievers in `dart/io`.

## Where To Go Next

- Modifying classic dynamics or kinematics: [dynamics.md](dynamics.md).
- Working on collision: [`dart/collision/AGENTS.md`](../../dart/collision/AGENTS.md)
  and [`design/README.md`](../design/README.md) for the native backend
  decisions.
- Adding classic constraints: [constraints.md](constraints.md).
- Understanding the DART 7 step: the architecture page and
  [profiling.md](profiling.md) for text-first step profiles.
- Key command: `pixi run test-all`; on Linux CUDA hosts also run
  `pixi run -e cuda test-all`.
