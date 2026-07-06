# DART 7 Simulation API Promotion Readiness Audit

## Status

Promotion readiness audit. This document is the PLAN-041 Workstream 1
deliverable: the **promotion contract and readiness audit** for the official
DART 7 simulation API. It
freezes the supported public subset, records the headers/modules to promote,
lists the public-looking internals to hide, maps the parity evidence that gates
the promotion claim, and tracks the concrete boundary blockers to clear.

The key boundary slices have landed: the simulation module builds by default,
installs only an explicit public-header allowlist, keeps EnTT/Taskflow out of
promoted header include paths, uses the `dart::simulation` namespace and
`dart-simulation` target, and promotes the Python import layout to
`dartpy.simulation.World` / `dartpy.World`. Remaining readiness work is cleanup,
negative smokes for retired names, and keeping the guards green.

Companion docs:

- API shape and rationale: [`simulation_cpp_api.md`](simulation_cpp_api.md)
  and [`simulation_python_api.md`](simulation_python_api.md).
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

Current gates:

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
  strict-final mode is the final local claim gate. In-tree parity references have been
  ratcheted to zero; any new main-branch `dart::simulation::World` parity
  dependency under `tests/unit/simulation/` fails the default
  blocker check. The former contact/constraint, skeleton-to-multibody, and
  world-parity rows now carry DART 7-only regression assertions; parity
  evidence belongs on `release-6.*` branches.
- `pixi run check-dart7-final-world-promotion` layers strict-final blocker
  removal with a required local `release-6.*` branch ref. It is intentionally
  not part of default lint while transition references are being retired, but it
  is the final claim gate for "parity came from `release-6.*` branches, not
  main's classic World implementation."

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

### E. Remaining promotion blockers

The public-surface leak blocker is cleared for the promoted simulation package.
The still-open blockers are cleanup and guard-hardening items:

- stale docs or user-facing snippets that still teach retired experimental or
  DART 6 paths;
- negative smokes for obsolete experimental headers, modules, targets, and
  aliases;
- any future parity claim that relies on main-branch DART 6 code instead of
  `release-6.*` branch evidence.

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

## Parity Evidence Map

Promotion of the public subset cannot be claimed until the DART 7 checkable parity
gates ([`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)) have
direct evidence. The gates and their required evidence (per-gate status is tracked
in PLAN-041, not here):

| Gate                         | Required evidence                                                         |
| ---------------------------- | ------------------------------------------------------------------------- |
| DART 7 model loading         | URDF/SDF/MJCF load with topology/DOF/transform/mass/collision.            |
| Rigid dynamics parity        | Shared open-chain scenes match the classic DART 6 path within tolerances. |
| Contact/constraint parity    | Contacts, friction, limits, motors, mimic/coupler, loop closures.         |
| Serialization/replay parity  | Topology/state/assets + record/replay round-trip within bounded error.    |
| Stable public API promotion  | Promoted headers hide ECS/components/solver/backend (boundary checks).    |
| Name-collision resolution    | `dart::simulation::World` owned by the promoted DART 7 facade.            |
| Core build/tests + packaging | Lint, build, tests, and package/export smokes for the touched scope.      |

## Operating state lives in the plan, not here

Per [`AGENTS.md`](AGENTS.md), this design doc does not own gates, per-workstream
status, or implementation handoff. The blocker workstream sequence and the
current per-gate parity status are tracked
in [`../plans/dashboard.md`](../plans/dashboard.md) and
[`../plans/041-official-simulation-api-promotion.md`](../plans/041-official-simulation-api-promotion.md)
(PLAN-041), with the DART 7 checkable parity gates in
[`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md).

## WS1 Acceptance

Workstream 1 is satisfied when: the supported public subset is frozen (above),
the hide-list is explicit (above), the package/header checks are executable
(`check-dart7-promotion-surface`, `check-simulation-public-header-smoke`,
`check-dart7-promotion-package-contract`,
`check-dart7-promotion-installed-package`, and `check-dartpy-import-layout`),
the remaining blockers are stated as durable design constraints (above), and
the parity-evidence map names each gate (above). Per-workstream sequencing and
per-gate status live in PLAN-041. Update this audit when the install rule,
package dependencies, header layout, or import layout changes.

## Verification

Docs-only edits use the docs-only gate set from `docs/ai/verification.md`.
Package/header/import gate changes also run the focused checker tests and
`pixi run check-lint`.
