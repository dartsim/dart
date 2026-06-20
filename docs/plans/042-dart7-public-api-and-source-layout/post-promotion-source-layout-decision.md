# PLAN-042 Post-Promotion Source Layout Decision

- Owner plan:
  [`../042-dart7-public-api-and-source-layout.md`](../042-dart7-public-api-and-source-layout.md)
- Operating state: `PLAN-042` in [`../dashboard.md`](../dashboard.md)
- Purpose: record the source-layout decision after the official DART 7
  simulation API promotion and keep follow-up moves narrow, auditable, and
  gate-backed.

## Decision

The current `dart/simulation` tree is the accepted post-promotion layout for the
official DART 7 World API. It is not the final whole-repo source taxonomy.

This means:

- Keep `dart::simulation::World`, `dart/simulation/world.hpp`,
  `dart/simulation/All.hpp`, the `dart-simulation` component, and
  `dartpy.simulation.World` / `dartpy.World` as the official promoted owners.
- Keep the physical tree guarded by the explicit CMake public-header allowlist
  and promotion-surface checks. A folder may exist in `dart/simulation` without
  becoming public API.
- Do not start a broad source move just because PR #2932 promoted the API.
  Future moves must be small, owner-specific PRs with a row in this record or
  the API/source-layout audit, a conflict preflight against the latest `main`,
  and the relevant package/header/import gates.
- Treat `release-6.*` branches as the parity and compatibility lane for DART 6
  behavior. Main remains the DART 7 clean-break line.
- Treat the DART 6.20+ port lane as the owner for remaining DART 6 parity
  needs. `main` keeps DART 6 legacy only as a frozen compatibility quarantine
  until eventual removal from the DART 7 public contract.

## Current Folder Classification

| Path / surface                          | Current role after promotion                                                               | Decision                                                                                                                                                                                                             | Required gate before widening exposure or moving files                                                                                                                                                |
| --------------------------------------- | ------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dart/simulation`                       | Official DART 7 simulation owner.                                                          | Keep as the module root for the promoted World API.                                                                                                                                                                  | Promotion/package/import gates stay green.                                                                                                                                                            |
| top-level promoted headers              | `World`, options, entity, forward declarations, export/version, and umbrella headers.      | Keep public and allowlisted.                                                                                                                                                                                         | `check-simulation-public-headers`, install smoke, case-insensitive header checks when names change.                                                                                                   |
| `body/`                                 | Storage-neutral public-looking body handles and options.                                   | Keep only allowlisted wrappers public; do not expose ECS ownership.                                                                                                                                                  | Boundary checks reject raw entity IDs, registry access, `comps/`, `ecs/`, and helper-base inheritance.                                                                                                |
| `frame/`                                | Frame handles used by the promoted World facade.                                           | Keep allowlisted lifecycle-safe handles public.                                                                                                                                                                      | Lifetime/ownership tests plus promoted-header boundary checks.                                                                                                                                        |
| `multibody/`                            | DART 7 multibody handles and options next to legacy `dart/dynamics`.                       | Keep current allowlisted surface; decide the long-term `dart/dynamics` relationship separately.                                                                                                                      | Cross-plan decision with PLAN-080 and release-6.\* parity evidence before moving or flattening more multibody concepts.                                                                               |
| `constraint/`                           | Curated loop-closure public surface.                                                       | Keep allowlisted extension points public.                                                                                                                                                                            | Constraint tests and API-boundary checks reject solver-registry or backend leakage.                                                                                                                   |
| `compute/` public allowlist             | Backend-neutral executor, stage metadata, and profiling value types.                       | Keep as an advanced public seam, not a solver/backend registry.                                                                                                                                                      | `check-compute-backend-boundaries`, promotion-header checks, and benchmark evidence for any wider compute API.                                                                                        |
| `compute/` internals and `compute/cuda` | Solver stages, kernels, batches, and private opt-in CUDA implementation.                   | Keep internal. Split graph metadata from kernels only in focused PRs with second-use evidence.                                                                                                                       | No Taskflow/CUDA/device/stream/memory-pool public types; CUDA remains opt-in and private.                                                                                                             |
| `diff/`                                 | Public step derivative/gradient headers plus opt-in rollout; final owner tied to PLAN-110. | Keep the narrow public subset; decide broader differentiable ownership with PLAN-110.                                                                                                                                | Torch-free import smoke, generated stubs/docs, differentiable tests, and no framework/backend dependency in default public headers.                                                                   |
| `detail/`                               | Solver schedules, deformable/VBD/Newton-barrier/variational internals.                     | Keep internal implementation owner.                                                                                                                                                                                  | No installed promoted headers and no public implementation-folder includes.                                                                                                                           |
| `comps/`                                | ECS component storage.                                                                     | Internal only.                                                                                                                                                                                                       | Negative header smoke and API-boundary checks reject public includes/signatures.                                                                                                                      |
| `ecs/`                                  | ECS access/helper templates.                                                               | Internal only.                                                                                                                                                                                                       | Public headers expose no entity IDs, registries, or ECS helper-base classes.                                                                                                                          |
| `common/`                               | Simulation-local support utilities.                                                        | Internal unless a reusable foundation proves a public owner.                                                                                                                                                         | Public-boundary review before any utility promotion.                                                                                                                                                  |
| `io/`                                   | Simulation serializers, loaders, and model bridges.                                        | Keep as private adapters; `dart/io` remains the public loading front door.                                                                                                                                           | IO docs/tests and installed-header/package smokes prove no duplicate public loader path.                                                                                                              |
| `space/`                                | State-space and vector-mapper support.                                                     | Keep internal until a public state model proves user-facing need.                                                                                                                                                    | State docs plus checks that mapper implementation and registry ownership do not leak.                                                                                                                 |
| `dart/dynamics`                         | DART 6 object model and legacy public multibody concepts.                                  | Frozen compatibility/quarantine lane now; eventual removal from the DART 7 public contract after DART 6.20+ and later release-6.\* port needs are settled. Surviving concepts need explicit DART 7 owners and names. | `check-dart7-legacy-freeze`; `release-6.*` evidence; `dart7-legacy-freeze: bugfix-port` for unavoidable parity ports only; migration notes; no return of DART 6 `World` as a main-branch common path. |

## Next Move Rules

Future source-layout PRs should follow this order:

1. Record the owner and public/internal outcome for the files being moved.
2. Fetch the latest `main` and preflight conflicts before touching paths with
   active solver work.
3. Move one owner at a time: public facade headers, internal solver/detail
   files, Python binding owner files, generated docs/stubs, or cleanup aliases.
4. Run the smallest gate that proves the move plus the common promotion gates.
5. Leave unrelated folder renovation to later PLAN-042 batches.

## Required Follow-Up Decisions

- Compute split: whether backend-neutral graph/profiling APIs stay under
  `dart/simulation/compute` or get a narrower public wrapper while kernels move
  deeper under implementation owners.
- IO ownership: which DART 7 model-loading APIs remain in `dart/io`, and which
  simulation adapters stay private behind `World`.
- State/space ownership: whether public state wrappers are needed, and if so
  whether they are `dart/simulation/state.hpp`, `dart/simulation/space.hpp`, or
  another owner.
- Differentiable helper home: coordinate C++ `diff/` and Python `dart.diff` /
  `dart.simulation.diff` with PLAN-110 before widening the public surface.
- Legacy dynamics removal staging: `dart/dynamics` and classic DART 6
  constraints are frozen on `main` as a compatibility quarantine while the DART
  6.20+ port lane and later `release-6.*` support branches finish needed parity.
  New public legacy symbols, bindings, or stubs require the local
  `dart7-legacy-freeze: bugfix-port` tag and should be ported to DART 7 owners
  instead whenever they are not release-6.\* bugfix parity.
- Negative smokes: keep adding checks for retired experimental headers, modules,
  targets, and aliases as the final public package shape hardens.
- Strict-final blocker cleanup: the default
  `check-dart7-world-promotion-blockers` gate passes, but the strict-final gate
  still reports promotion-guard-script buckets for the experimental build option
  and classic render-world binding. Retire those only in the final package/import
  cleanup slice, when the corresponding guard scripts and tests can move from
  transition detection to final negative checks.

## Verification

For this docs/planning record:

- `pixi run lint-md`
- `pixi run check-lint-md`
- `pixi run check-docs-policy`
- `pixi run check-lint-spell`
- `pixi run lint`

For implementation PRs derived from this record, select by touched scope and
include the promotion gates that protect the public layout:

- `pixi run check-simulation-public-headers`
- `pixi run check-dart7-promotion-package-contract`
- `pixi run check-dart7-promotion-installed-package`
- `pixi run check-dartpy-import-layout`
- `pixi run check-dart7-world-promotion-blockers`
- `pixi run check-dart7-legacy-freeze`
- `pixi run check-dart7-legacy-freeze-meta`
- `pixi run check-dart7-final-world-promotion` before claiming final parity
  independence from main-branch DART 6 code
