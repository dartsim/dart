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

## Python Demo Catalog

DART has one demo application, `python/examples/demos/`, launched through
`dartpy.viewer`. Python declares worlds, key actions, light step hooks, and
declarative `Control` panels. C++ owns the docking workspace, navigator,
inspector, tools, profiler, log console, scene lifecycle, and headless list,
cycle, and capture. Heavy controllers and models stay native; Python has no
ImGui drawing API.

| Piece | Location and target | Boundary |
| --- | --- | --- |
| App host | `dart/gui/app/` → `dart-gui-app` | Private static target in `dart::gui::app`, excluded from `ALL` unless a consumer needs it; never installed or exported |
| GUI-free models | `python/native/demo_models/` → `dart-demo-models` | Private static target in `dart_demos`, shared by dartpy and C++ model tests; builds with the GUI off |
| Host bindings | `python/dartpy/viewer/` → `dartpy.viewer` | Compiled only when `dart-gui-app` exists; names follow the DART 7 Python style |
| Model bindings | `python/dartpy/demos/` → `dartpy._demos` | Private native helpers for the source-tree catalog |
| Scenes | `python/examples/demos/scenes/`, `registry.py`, `__main__.py` | Python factories and catalog registration; the catalog is not included in wheels |

`dart-gui-app` links `dart-gui` and `dart-demo-models`. Its placement under
`dart/` makes it available before `tests/` and `python/` are configured.
The model library is added immediately after `dart/`; IO-dependent sources
are included only when `dart-utils` and `dart-utils-urdf` exist, and the
point-cloud sensor requires OctoMap. Both libraries link into the dartpy
extension without adding installed headers, package components, or a
Gazebo-facing API.

The host requires `IMGUI_HAS_DOCK` in the copied ImGui header. Bundled builds
use the docking release; a vanilla system ImGui compiles out the host and
`dartpy.viewer` while retaining `dart-gui` and dartpy. The Python catalog then
exits with a rebuild hint. See [build guidance](building.md#filament-gui),
[callback ownership](python-bindings.md#viewer-app-callbacks),
[scene rules](../../examples/README.md#adding-catalog-scenes), and
[catalog verification](testing.md#demo-catalog-verification).

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
