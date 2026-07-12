# DART 6.20 Architecture

This document owns the release branch's component map and change-routing
boundaries. DART 6.20 is a compatibility lane: architecture changes must
preserve C++17, installed APIs, package components, ABI-sensitive interfaces,
and downstream Gazebo/gz-physics behavior unless explicitly approved.

## Component Layers

| Layer | Primary paths | Responsibility and boundary |
| --- | --- | --- |
| Foundations | `dart/common/`, `dart/math/` | Shared utilities, resources, logging, memory, geometry, and math used by higher layers |
| Multibody model | `dart/dynamics/` | Skeletons, bodies, joints, shapes, aspects, kinematics, and dynamics state |
| Collision | `dart/collision/` | Collision detector abstraction plus DART-native, FCL, Bullet, and ODE integrations selected through existing components |
| Constraints | `dart/constraint/` | Contact and joint constraints, LCP-based solving, and solver-owned per-step state |
| Simulation | `dart/simulation/` | `World` ownership, stepping, time integration, and orchestration of collision and constraints |
| Model loading | `dart/utils/` | DART 6 loaders and URDF/SDF/SKEL/MJCF parsing; public loading remains under `dart::utils` |
| GUI | `dart/gui/osg/` | OSG viewers, world nodes, ImGui integration, and offscreen capture |
| Python | `python/dartpy/` | pybind11 bindings that expose the supported DART 6 API to `dartpy` |
| Downstream integration | package exports and Gazebo Pixi environment | Installed headers/targets and pinned gz-physics/gz-sim compatibility gates |

The usual runtime flow is: a `dart::utils` loader creates `Skeleton` objects;
a `simulation::World` owns and steps them; collision detectors produce contacts;
the constraint solver resolves contacts and joint constraints; dynamics state is
integrated; OSG or dartpy presents the result. Keep ownership at these existing
boundaries instead of adding task-local bypasses.

## Compatibility Boundaries

- Public headers and exported CMake components are release API. Moving an
  implementation does not authorize removing its installed compatibility path.
- Collision, constraint, and `World::step` changes can alter default physics and
  downstream Gazebo behavior even when focused unit tests pass.
- Parser changes must preserve DART 6 file formats, `dart::utils` names, resource
  retrieval, and installed dependencies.
- Python changes use pybind11 and the existing camelCase DART 6 surface. DART 7
  nanobind or snake_case assumptions do not apply.
- GUI changes use the OSG path and require artifact inspection when rendering,
  interaction, or capture behavior changes.

## Change Routing And Gates

| Change | Read next | Minimum evidence |
| --- | --- | --- |
| Build, package, or dependency | [`building.md`](building.md), [`build-system.md`](build-system.md) | Configure/build plus affected package/component checks |
| Dynamics or simulation | [`testing.md`](testing.md), [`../ai/verification.md`](../ai/verification.md) | Focused C++ tests, broad release gate, behavior/visual evidence when applicable |
| Collision or constraints | [`testing.md`](testing.md), [`ci-cd.md`](ci-cd.md) | Focused tests plus `pixi run -e gazebo test-gz` when downstream behavior can change |
| Parser or model loading | [`io-parsing.md`](io-parsing.md) | Focused loader tests and downstream/package evidence as applicable |
| dartpy binding | [`python-bindings.md`](python-bindings.md) | `pixi run build-py-dev` and `pixi run test-py` |
| OSG GUI or capture | [`testing.md`](testing.md), [`../ai/verification.md`](../ai/verification.md) | Runnable scene, captured artifact, and image/text verdict appropriate to the claim |
| Backport or release workflow | [`release-management.md`](release-management.md) | Apply/adapt/omit comparison plus release-target gates |

## DART 7 Comparison Boundary

`main` uses a broader DART 7 architecture and newer implementation conventions.
Use it to find candidate fixes or design evidence, then prove the release change
against this tree. Do not create DART 7-only solver/backend layers, C++23 APIs,
nanobind bindings, or `dart::io` loading paths merely to make a backport match.
