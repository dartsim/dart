# DART 7 Simulation API Promotion Readiness Audit

## Status

Proposal. This document is the PLAN-041 Workstream 1 deliverable: the **promotion
contract and readiness audit** for moving `dart::simulation::experimental` to the
official DART 7 simulation API. It freezes the supported public subset, records
the headers/modules to promote, lists the public-looking internals to hide, maps
the parity evidence that gates the promotion claim, and tracks the concrete
boundary blockers to clear.

This step edits docs and the boundary inventory only; it does not move source
files. Source moves are PLAN-041 Workstream 9.

Companion docs:

- API shape and rationale: [`simulation_experimental_cpp_api.md`](simulation_experimental_cpp_api.md)
  and [`simulation_experimental_python_api.md`](simulation_experimental_python_api.md).
- Operating state, sequencing, and gates: `PLAN-041` in
  [`../plans/dashboard.md`](../plans/dashboard.md) and
  [`../plans/041-official-simulation-api-promotion.md`](../plans/041-official-simulation-api-promotion.md).
- Public/internal API policy: [`../onboarding/api-boundaries.md`](../onboarding/api-boundaries.md).
- The `dart::simulation::World` name-collision transaction is tracked
  separately under PLAN-041 Workstream 4.

This audit owns inventory and readiness state, not API-shape rationale (which
lives in the companion shape docs) and not priority/horizon (which lives in the
dashboard).

## Why an audit before a facade

Today there is **no public/internal split** in the experimental tree. The
component installs every header by directory glob and depends on its ECS and
task-graph libraries publicly, so the "experimental API" currently leaks its
entire implementation. Promotion cannot be claimed until the supported subset is
frozen and the implementation surface is hidden. This audit makes that subset and
that hide-list explicit so the facade work (Workstreams 2/3/5) has a checklist.

## Current Public-Surface Audit (evidence)

### A. The install rule ships the whole tree as public

`dart/simulation/experimental/CMakeLists.txt`:

```cmake
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/dart/simulation/experimental
  COMPONENT headers
  FILES_MATCHING
  PATTERN "*.hpp"
)
```

This installs **all 127 `*.hpp`** under `experimental/`, including `detail/`,
`comps/`, `ecs/`, and `compute/` implementation headers. There is no public-header
allowlist today.

### B. ECS and task-graph libraries are public package dependencies

`dart/simulation/experimental/CMakeLists.txt`:

```cmake
add_component_dependency_packages(${PROJECT_NAME} ${component_name} EnTT Taskflow)
```

`EnTT` and `Taskflow` are declared as **public** component dependency packages, so
a downstream consumer of the promoted API would be forced to find and carry them.
A promoted public API must not require either.

### C. The world handle leaks ECS storage

`dart/simulation/experimental/world.hpp`:

- `entt::registry& getRegistry();` and the `const` overload (lines ~449–452) are a
  raw storage escape hatch. The shape doc already classifies this as
  implementation-only and excluded from promotion.
- The header includes `<entt/entt.hpp>` directly.

### D. Header inventory by subdirectory (127 total)

Counts are a snapshot and drift as the tree grows; recompute the current total
with `find dart/simulation/experimental -name '*.hpp' | wc -l`. The dedicated
promotion-surface audit script (added with the WS1/WS5 facade-prep tooling)
additionally reports per-header classification and the (transitive) ECS-leak
set.

| Subdir        | hpp | Disposition | Notes                                                                                                                                                                                                                   |
| ------------- | --- | ----------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| _(top level)_ | 6   | PROMOTE\*   | `world`, `world_options`, `world_sync_stage`, `fwd`, `export`, `version`. `world.hpp` must drop `getRegistry`/`entt`.                                                                                                   |
| `body/`       | 7   | PROMOTE\*   | `rigid_body`, `collision_body`, `contact`, `deformable_body` (+ options) handles; some include `entt`.                                                                                                                  |
| `multibody/`  | 5   | PROMOTE\*   | `multibody`, `link`, `joint` handles; include `entt` today.                                                                                                                                                             |
| `frame/`      | 3   | PROMOTE\*   | `frame` handle; includes `entt` today.                                                                                                                                                                                  |
| `constraint/` | 5   | PROMOTE\*   | `loop_closure` handle + spec/policy/residual; hide the `*_family`/`*_residual` internals if implementation-only.                                                                                                        |
| `space/`      | 4   | MIXED       | `StateSpace` PROMOTE; `auto_mapper`/`component_mapper`/`vector_mapper` HIDE (registry mappers).                                                                                                                         |
| `common/`     | 9   | MIXED       | `constants`/`exceptions`/`diagnostics` may be public; `ecs_utils` HIDE; `macros`/`assert`/`logging`/`profiling`/`type_list` are support headers to keep internal unless needed.                                         |
| `comps/`      | 16  | HIDE        | ECS component storage. Never public.                                                                                                                                                                                    |
| `compute/`    | 20  | HIDE\*\*    | Executors/kernels/backends. Only backend-neutral extension points (`ComputeExecutor`, `WorldStepPipeline`, `WorldStepStage`, profiles, metadata) are promotion candidates, and only with benchmark + boundary evidence. |
| `ecs/`        | 4   | HIDE        | Entity-object base machinery.                                                                                                                                                                                           |
| `io/`         | 11  | HIDE\*\*\*  | Binary serializers. `World::saveBinary`/`loadBinary` is the public surface; the serializer headers stay internal.                                                                                                       |
| `detail/`     | 33  | HIDE        | Already `detail/` (solvers: boxed-LCP, rigid IPC, deformable VBD/IPC, elasticity). Never public.                                                                                                                        |
| `diff/`       | 4   | SEPARATE    | Differentiable simulation (PLAN-110), opt-in behind `DART_BUILD_DIFF`; not part of the core promotion subset.                                                                                                           |

`*` PROMOTE handles currently leak `entt` (include or signature) and need
opaque-ownership/pimpl or include cleanup in the facade step before promotion.

`**` Default to HIDE; promote only the explicitly backend-neutral compute concepts
the shape doc allows, gated by benchmark + `check-api-boundaries`.

`***` Public serialization is the `World` method surface, not the serializer
headers.

### E. Headers that currently leak ECS in the would-be public set

Non-`detail/` headers that include `entt`/`registry` or expose `getRegistry`
(must be cleaned or hidden before promotion):

- Promote-target handles to **clean**: `world.hpp`, `body/rigid_body.hpp`,
  `body/collision_body.hpp`, `frame/frame.hpp`, `frame/free_frame.hpp`,
  `frame/fixed_frame.hpp`, `multibody/{joint,link,multibody}.hpp`,
  `constraint/loop_closure.hpp`. `frame/free_frame.hpp` and
  `frame/fixed_frame.hpp` need cleanup even once their `entt::entity`
  constructors are removed, because they still inherit
  `EntityObjectWith<...comps::...>` via `frame.hpp`; they are clean only after
  the shared `Frame`/`EntityObjectWith` base is de-ECS'd.
- Internals to **hide** (stop installing): `common/ecs_utils.hpp`, all `comps/*`,
  `ecs/*`, `io/{serializer,category_serializer,binary_io}.hpp`,
  `space/{auto_mapper,component_mapper,vector_mapper}.hpp`,
  `compute/{multibody_dynamics,variational_integration,world_kinematics_graph}.hpp`.

## Frozen Supported Public Subset (PROMOTE)

The supported DART 7 public simulation subset, by concept (names per `fwd.hpp`;
final spelling decided in the facade step):

- **World lifecycle**: `World`, `WorldOptions`, `WorldSyncStage`; design/sim-mode
  lifecycle; `step`/`sync`/`updateKinematics`; `saveBinary`/`loadBinary`;
  `setGravity`/`setTimeStep`.
- **Object handles**: `RigidBody`, `Multibody`, `Link`, `Joint`, `Frame`,
  `FreeFrame`, `FixedFrame`, `LoopClosure`, `DeformableBody`, `Contact`.
- **Construction value objects**: `RigidBodyOptions`, `JointSpec`/`JointOptions`,
  `LinkOptions`, `FreeFrameOptions`/`FixedFrameOptions`, `LoopClosureSpec`,
  `LoopClosureRuntimePolicy`, `WorldOptions`, `MultibodyOptions`, deformable
  options/material/boundary structs.
- **State**: `StateSpace`; state/control vector get/set.
- **Diagnostics**: `LoopClosureResidual`, deformable solver diagnostics value
  types.
- **Capability selection (method-named, value-object)**: `MultibodyOptions`
  integration family; `RigidBodySolver`/`ContactSolverMethod` (review each name to
  confirm it is method-named, not engine-named, per the shape doc).
- **Backend-neutral compute extension points (conditional)**: `ComputeExecutor`,
  `ParallelExecutor`, `WorldStepPipeline`, `WorldStepStage`, execution profiles,
  stage metadata — promote only with benchmark + boundary evidence.

## Internals to Hide (HIDE)

Never part of the promoted public contract: `comps/*`, `ecs/*`, `detail/*`,
`common/ecs_utils.hpp`, the `space/` registry mappers, the `io/` serializer
internals, `compute/*` backend/kernel implementations (executors, kernels,
device/batch backends), and `World::getRegistry()`. The `diff/` subtree is a
separate opt-in capability (PLAN-110), not core-promotion surface.

## Promotion Blockers (durable design constraints)

The audit above identifies the surface promotion must clear. As durable design
constraints (per-workstream tracking and status live in the plan, see below):

- **Install boundary:** replace the `*.hpp` install glob with an explicit
  public-header allowlist; make `EnTT`/`Taskflow` private dependencies; make the
  promoted baseline build without `DART_BUILD_SIMULATION_EXPERIMENTAL`.
- **Handle de-ECS:** no promoted public signature or include exposes
  `entt`/registry/component types; remove or internalize `World::getRegistry()`.
- **Boundary enforcement:** boundary checks reject EnTT/registry/`comps`/`ecs`/
  `detail`/backend leakage from promoted headers, with negative
  installed-package smokes.
- **Name collision:** clear `dart::simulation::World` via an explicit
  transaction before the promoted facade owns the official namespace.
- **Python facade:** confirm the `dartpy.simulation` path and the `dartpy.World`
  decision with no duplicate nanobind registration.

## Operating state lives in the plan, not here

Per [`AGENTS.md`](AGENTS.md), this design doc does not own gates, per-workstream
status, or implementation handoff. The blocker workstream sequence, current
status, and the parity-evidence gates that block the promotion claim are tracked
in [`../plans/dashboard.md`](../plans/dashboard.md) and
[`../plans/041-official-simulation-api-promotion.md`](../plans/041-official-simulation-api-promotion.md)
(PLAN-041), with the DART 7 checkable parity gates in
[`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md).

## WS1 Acceptance

Workstream 1 is satisfied when: the supported public subset is frozen (above), the
hide-list is explicit (above), the boundary blockers are enumerated as a checklist
(above), and the parity-evidence map names each gate. Subsequent workstreams turn
each checklist item into an implementation PR. Update this audit when the install
rule, package dependencies, header layout, or parity status changes.

## Verification

Docs-only edits use the docs-only gate set from `docs/ai/verification.md`:
`pixi run lint-md`, `pixi run check-lint-md`, `pixi run check-lint-spell`,
`pixi run check-docs-policy`, and `pixi run lint`.
