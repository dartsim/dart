# DART Architecture

This document owns the development branch's component map and change-routing
boundaries. Architecture changes on `main` follow the compatibility rules in
`AGENTS.md`: preserve C++17, installed APIs, package components, and the C++
API and behavior that Gazebo/gz-physics and gz-sim use. ABI-sensitive
interfaces and default behavior change only with maintainer approval.

## Component Layers

| Layer | Primary paths | Responsibility and boundary |
| --- | --- | --- |
| Foundations | `dart/common/`, `dart/math/` | Shared utilities, resources, logging, memory, geometry, and math used by higher layers |
| SIMD | `dart/simd/` | Header-only SIMD abstraction (compile-time backend selection via `config.hpp` macros — scalar/SSE/AVX/AVX2/AVX-512/NEON — plus dynamically sized matrix/vector types, Eigen bridges, and geometry/math kernels) used by performance-sensitive layers; validated by the `CI SIMD Multi-Arch` workflow |
| Multibody model | `dart/dynamics/` | Skeletons, bodies, joints, shapes, aspects, kinematics, and dynamics state |
| Collision | `dart/collision/` | Collision detector abstraction plus the DART-owned `dart`, FCL, ODE, and other optional implementations selected through existing components |
| Constraints | `dart/constraint/`, `dart/lcpsolver/` | Contact and joint constraints, LCP-based solving (Lemke/Dantzig in `dart/lcpsolver/`), and solver-owned per-step state |
| Optimization | `dart/optimizer/` | The retained gradient-based optimizer interfaces used by dynamics and IK |
| Simulation | `dart/simulation/` | `World` ownership, stepping, time integration, and orchestration of collision and constraints |
| Model loading | `dart/utils/` | DART loaders and URDF/SDF/SKEL/MJCF parsing; public loading remains under `dart::utils` |
| GUI | `dart/gui/` | Filament viewers, world nodes, ImGui integration, and offscreen capture |
| Python | `python/dartpy/` | nanobind bindings that expose the supported DART API to `dartpy` |
| Downstream integration | package exports and Gazebo Pixi environment | Installed headers/targets and pinned gz-physics/gz-sim compatibility gates |

The usual runtime flow is: a `dart::utils` loader creates `Skeleton` objects;
a `simulation::World` owns and steps them; collision detectors produce contacts;
the constraint solver resolves contacts and joint constraints; dynamics state is
integrated; Filament or dartpy presents the result. Keep ownership at these existing
boundaries instead of adding task-local bypasses.

The private `dart/gui/app/` host (`dart-gui-app`) owns the demo window,
docking workspace, scene lifecycle, and command-line dispatch. It links
`dart-gui` and the GUI-free `dart-demo-models`; it is never installed or exported.
The C++ demo catalog uses compatibility shims in `examples/demos/`. A vanilla
system ImGui compiles out this host while retaining `dart-gui` and dartpy.

### World step stages

`World::step(bool)` stays the exported, non-virtual entry point: Gazebo calls
it, the allocation shim interposes it, and Callgrind collects inside it. It
dispatches to private, non-virtual stages that share a private
`World::StepContext`, defined in `World.cpp` and kept on the stack for one call:

- `beginStep` prepares simulation mode, resets the frame allocator, reads the
  deactivation flag once and mirrors it onto the constraint solver, and, with
  deactivation on, wakes resting skeletons after edits made between steps.
- With deactivation off, `stepWithoutDeactivation` runs the whole step.
- With deactivation on, `tryAllRestingFastPath` records whether a custom
  `BodyNodeCollisionFilter` lets skeletons sleep this step, a flag later stages
  read. When the last solve found no contacts, it runs the all-resting
  readiness scan, which can clear residual forces and commands and wake resting
  skeletons. If every mobile skeleton still rests and nothing changed since the
  last step, it advances time and the frame count without integrating or
  solving, clears resting skeletons' accelerations under such a filter, and
  returns true. Otherwise `advanceVelocities`, `solveConstraints`,
  `advancePositions`, and `finishDeactivationStep` run in that order.

Every path returns through the end of `World::step`, the one completed-step
boundary. The constraint solver runs once per step outside the fast path and
never on it. `StepContext` carries nothing to the next step; it keeps the
solve's collision group and filter alive until `World::step` returns.

## Compatibility Boundaries

- Public headers and exported CMake components are release API. Moving an
  implementation does not authorize removing its installed compatibility path.
- Collision, constraint, and `World::step` changes can alter default physics and
  downstream Gazebo behavior even when focused unit tests pass.
- Parser changes must preserve supported file formats, `dart::utils` names, resource
  retrieval, and installed dependencies.
- Python changes use nanobind; new APIs follow the DART 7 Python style while
  the PLAN-624 rename is in progress.
- GUI changes use the Filament path and require artifact inspection when rendering,
  interaction, or capture behavior changes.

### Performance compatibility

The gz-physics dartsim plugin subclasses `OdeCollisionDetector` and
`ContactSurfaceHandler`, casts the solver to `BoxedLcpConstraintSolver`, and
uses `BodyNode::moveTo` detach/reattach flows. Preserve the vtables of
`CollisionDetector`, `OdeCollisionDetector`, `ContactSurfaceHandler`, and
`BoxedLcpConstraintSolver`. Detector selection and collision-result access are
covered by the [downstream contract](../design/dart6_collision_backends.md#downstream-contract).

Simulation threads remain opt-in through `World::setNumSimulationThreads`,
with a default of one; do not assume Gazebo enables them. Behavior-preserving
optimizations retain each detector's final-state hash and contact, pair, and
resting counts on the guard scenes. Behavior-changing work needs a separate
PR with old/new guard rows, tolerance rationale, and approved rebaselining.

Keep structure-of-arrays or aligned scratch behind implementation-private
storage or in function-local state. Run `pixi run test-eigen-overalignment`
when alignment changes. Do not add `-march` flags to exported targets.

## Change Routing And Gates

| Change | Read next | Minimum evidence |
| --- | --- | --- |
| Build, package, or dependency | [`building.md`](building.md), [`build-system.md`](build-system.md) | Configure/build plus affected package/component checks |
| Dynamics or simulation | [`testing.md`](testing.md), [`../ai/verification.md`](../ai/verification.md) | Focused C++ tests, broad release gate, behavior/visual evidence when applicable |
| Collision or constraints | [`testing.md`](testing.md), [`ci-cd.md`](ci-cd.md) | Focused tests plus `pixi run -e gazebo test-gz` when downstream behavior can change |
| Parser or model loading | [`io-parsing.md`](io-parsing.md) | Focused loader tests and downstream/package evidence as applicable |
| dartpy binding | [`python-bindings.md`](python-bindings.md) | `pixi run build-py-dev` and `pixi run test-py` |
| Filament GUI or capture | [`testing.md`](testing.md), [`../ai/verification.md`](../ai/verification.md) | Runnable scene, captured artifact, and image/text verdict appropriate to the claim |
| Release workflow | [`release-management.md`](release-management.md) | Release-target gates |
