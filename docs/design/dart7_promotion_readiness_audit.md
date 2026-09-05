# DART 7 Simulation API Promotion Readiness Audit

## Purpose

This audit records the official DART 7 simulation API's supported public subset,
header/module inventory, internals to hide, and durable package and boundary
constraints. It does not establish simulation or release readiness.

Companion docs:

- API shape and rationale: [`simulation_cpp_api.md`](simulation_cpp_api.md)
  and [`simulation_python_api.md`](simulation_python_api.md).
- Operating state and work selection: [`../plans/dashboard.md`](../plans/dashboard.md).
- Milestone scope and acceptance evidence:
  [`PLAN-040`](../plans/040-dart7-release-hardening.md).
- API promotion status, sequencing, checkpoints and installed workflows:
  [`PLAN-041`](../plans/041-official-simulation-api-promotion.md).
- Public/internal API policy: [`../onboarding/api-boundaries.md`](../onboarding/api-boundaries.md).

## Why an audit before a facade

The promotion needed an audit before a facade because the original staging tree
had no public/internal split: it installed every header by directory glob and
required ECS/task-graph dependency discovery from downstream consumers. That
specific leakage is now guarded under the promoted namespace and package target.
This audit keeps the supported subset and hide-list explicit so future cleanup
does not regress those boundaries.

## Current Public-Surface Audit (evidence)

### A. Installed headers are allowlisted

`dart/simulation/CMakeLists.txt`:

```cmake
set(
  DART_SIMULATION_PUBLIC_HEADERS
  ${dart_simulation_public_headers_toplevel}
  ${dart_simulation_public_headers_body}
  ${dart_simulation_public_headers_multibody}
  ${dart_simulation_public_headers_frame}
  ${dart_simulation_public_headers_constraint}
  ${dart_simulation_public_headers_compute}
  ${dart_simulation_public_headers_diff}
)
```

Only that allowlist is installed through `install(FILES ...)`. The prior
recursive `install(DIRECTORY ... PATTERN "*.hpp")` rule is gone, so `comps/`,
`ecs/`, most `compute/`, `common/`, `detail/`, `io/`, and `space/` headers stay
out of the installed package unless a row is deliberately promoted.

### B. Implementation dependencies are private

`dart/simulation/CMakeLists.txt`:

```cmake
target_link_libraries(
  ${target_name}
  PUBLIC dart Eigen3::Eigen
  PRIVATE
    EnTT::EnTT
    spdlog::spdlog
    ${PROJECT_NAME}-collision-native
    ${PROJECT_NAME}-io
)
```

`EnTT`, `Taskflow`, and `spdlog` are implementation dependencies. Shared builds
do not register them as public component dependency packages. Static builds
still register them only under `if(NOT BUILD_SHARED_LIBS)` so exported
`$<LINK_ONLY:...>` entries can resolve without adding include or usage
requirements to promoted headers.

### C. Promoted headers are EnTT/Taskflow-clean

The original blocker was `world.hpp` exposing `entt::registry` through
`getRegistry()` and including `<entt/entt.hpp>` directly. That storage escape
hatch has been replaced by opaque world storage, and promoted headers are now
checked transitively.

Boundary verification mechanisms:

- `pixi run check-dart7-promotion-surface` runs
  `scripts/audit_dart7_promotion_surface.py --strict`, classifies promotion
  targets, follows their simulation include closure, rejects EnTT/Taskflow,
  ECS, `comps`, and internal-header leaks, and cross-checks the CMake install
  allowlist.
- `pixi run check-simulation-public-header-smoke` builds a translation unit
  that includes every allowlisted public header while poison EnTT/Taskflow
  headers are ahead of the real dependency include paths.
- `pixi run check-dart7-promotion-package-contract` statically guards the
  package/CMake facts above: the World stack remains non-optional on
  `main`, opt-in diff/CUDA subfeatures default-off, no recursive public install,
  and no unconditional private dependency-package leak.
- `pixi run check-dart7-promotion-installed-package` configures a minimal local
  install build, installs to a temporary prefix, compiles and runs a downstream
  CMake project against the allowlisted public headers, and verifies selected
  ECS/internal headers are absent from the installed prefix.
- `pixi run check-dart7-world-promotion-blockers` keeps the
  C++/package blocker inventory executable: experimental namespace/include-path
  uses, staged `dart-simulation-experimental` package names, and related transition
  references must stay in named transition buckets, and those code/build/test
  bucket counts may not grow. Its
  strict-final mode rejects remaining transition references.

The [API-boundary policy](../onboarding/api-boundaries.md) owns promotion-check
requirements and points to the checker-transition contract. Consult that owner
for final-promotion checks and release-reference handling.

### D. Header inventory ownership

The executable inventory source of truth is
`scripts/audit_dart7_promotion_surface.py`, not a hand-maintained count table.
Its current promotion groups are:

- top-level `world`, options, sync stage, entity, fwd, export, and version
  headers;
- public handle/value-object headers under `body/`, `multibody/`, `frame/`, and
  `constraint/`;
- the explicitly promoted backend-neutral compute profile/metadata headers; and
- the small diff value-type include closure, with `diff/rollout.hpp` installed
  only when `DART_BUILD_DIFF=ON`.

Everything else under `comps/`, `ecs/`, `detail/`, `io/`, `space/`, `common/`,
most of `compute/`, and most of `diff/` remains internal unless it is added to
that script and the CMake allowlist together.

## Frozen Supported Public Subset (PROMOTE)

The supported DART 7 public simulation subset, by concept (names per the
promoted facade):

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

- **Install boundary:** keep the explicit public-header allowlist, private
  EnTT/Taskflow/spdlog dependency shape, and package-contract checker green on
  the DART 7 target/component.
- **Handle de-ECS:** no promoted public signature or include exposes
  `entt`/registry/component types; `World::getRegistry()` must stay internal or
  absent from the promoted contract.
- **Boundary enforcement:** boundary checks reject EnTT/registry/`comps`/`ecs`/
  `detail`/backend leakage from promoted headers, with negative
  installed-package smokes.
- **Name collision:** keep `dart::simulation::World` owned by the promoted DART
  7 facade; do not re-export the classic world on `main`.
- **Python facade:** keep `dartpy.simulation.World` / `dartpy.World` as one class
  identity with no duplicate nanobind registration and no
  `dartpy.simulation_experimental` runtime/stub surface.

## Readiness Ownership

Use [PLAN-040](../plans/040-dart7-release-hardening.md) for physical correctness
oracles, the role of DART 6 comparisons, milestone dependencies and acceptance
criteria. Use [PLAN-041](../plans/041-official-simulation-api-promotion.md) for
promotion blockers, checkpoint and installed-workflow evidence, and completion
status. Passing the boundary checks described here does not establish completion
of those plans.

## Maintenance

Update this audit when the supported subset, install rule, package dependencies,
header layout, or import layout changes. Keep readiness status and acceptance
criteria in their owning plans.

## Verification

Docs-only edits use the docs-only gate set from `docs/ai/verification.md`.
Package/header/import gate changes also run the focused checker tests and
`pixi run check-lint`.
