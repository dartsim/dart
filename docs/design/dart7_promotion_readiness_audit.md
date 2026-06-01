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

This installs **all 117 `*.hpp`** under `experimental/`, including `detail/`,
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

### D. Header inventory by subdirectory (117 total)

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
| `compute/`    | 17  | HIDE\*\*    | Executors/kernels/backends. Only backend-neutral extension points (`ComputeExecutor`, `WorldStepPipeline`, `WorldStepStage`, profiles, metadata) are promotion candidates, and only with benchmark + boundary evidence. |
| `ecs/`        | 4   | HIDE        | Entity-object base machinery.                                                                                                                                                                                           |
| `io/`         | 9   | HIDE\*\*\*  | Binary serializers. `World::saveBinary`/`loadBinary` is the public surface; the serializer headers stay internal.                                                                                                       |
| `detail/`     | 27  | HIDE        | Already `detail/` (solvers: boxed-LCP, rigid IPC, deformable VBD/IPC, elasticity). Never public.                                                                                                                        |
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
  `body/collision_body.hpp`, `frame/frame.hpp`, `multibody/{joint,link,multibody}.hpp`,
  `constraint/loop_closure.hpp`.
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

## Blockers To Clear Before Promotion (checklist)

Tracked against PLAN-041 workstreams; each needs its own implementation PR:

- [ ] **B-AUDIT-1 (WS2)** Replace the install glob with an explicit public-header
      allowlist (or move internals under `detail/`/non-installed paths) so only the
      PROMOTE subset installs.
- [ ] **B-AUDIT-2 (WS2)** Make `EnTT` and `Taskflow` **private** component
      dependencies; the promoted public headers must not include or require either.
- [ ] **B-AUDIT-3 (WS2)** Make the promoted baseline build in default builds
      (remove the `DART_BUILD_SIMULATION_EXPERIMENTAL` gate for the promoted
      subset); keep reduced-build/package smokes while the option still exists.
- [ ] **B-AUDIT-4 (WS5)** Clean the promote-target handles so no public signature
      or include exposes `entt`/registry/components (opaque ownership / pimpl /
      forward-declared handles).
- [ ] **B-AUDIT-5 (WS5)** Remove or internalize `World::getRegistry()` from the
      promoted header (keep only behind an internal/test-only path).
- [ ] **B-AUDIT-6 (WS3)** Extend `scripts/check_api_boundaries.py` +
      `generate_api_boundary_inventory.py` to **reject** EnTT/registry/`getRegistry`/
      `comps`/`ecs`/`detail`/solver-registry/backend leakage from the _promoted_
      headers, and add negative installed-package smokes for forbidden headers and
      the obsolete `simulation-experimental` target.
- [ ] **B-AUDIT-7 (WS4)** Resolve the `dart::simulation::World` name collision per
      the Workstream 4 transaction design before the promoted facade owns the
      official namespace.
- [ ] **B-AUDIT-8 (WS6)** Confirm the `dartpy.simulation` import path, the
      `dartpy.World` decision, and no duplicate nanobind class registration.

## Parity Evidence Map (gates the promotion claim)

Promotion of the public subset cannot be **claimed** until the DART 7 checkable
parity gates (`docs/onboarding/release-roadmap.md`) have direct evidence. Current
state (this audit references in-flight work for context only; it does not own
those PRs):

| Gate (release roadmap)       | Required evidence                                                      | Status (as of this audit)                      |
| ---------------------------- | ---------------------------------------------------------------------- | ---------------------------------------------- |
| Experimental model loading   | URDF/SDF/MJCF/SKEL load with topology/DOF/transform/mass/collision.    | In review (PLAN-080 model-loader work).        |
| Rigid dynamics parity        | Shared open-chain scenes match classic DART 6 within tolerances.       | Harness in progress (PLAN-080 B2).             |
| Contact/constraint parity    | Contacts, friction, limits, motors, mimic/coupler, loop closures.      | Unified boxed-LCP solver in review (PLAN-080). |
| Serialization/replay parity  | Topology/state/assets + record/replay round-trip within bounded error. | Not yet started (owed; B4).                    |
| Stable public API promotion  | Promoted APIs hide ECS/components/solver/backend (boundary checks).    | Blocked on B-AUDIT-1..6 above.                 |
| Name-collision resolution    | `dart::simulation::World` cleared via explicit transaction.            | Design proposed (WS4).                         |
| Core build/tests + packaging | Lint, build, tests, package/export smokes per touched scope.           | Per-PR.                                        |

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
