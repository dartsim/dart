# PLAN-042 API And Source Layout Audit

- Owner plan:
  [`../042-dart7-public-api-and-source-layout.md`](../042-dart7-public-api-and-source-layout.md)
- Operating state: `PLAN-042` in [`../dashboard.md`](../dashboard.md)
- Purpose: classify the current C++ folders, Python modules, generated surfaces,
  and package exports for the DART 7 official World API and any remaining source
  cleanup.

## Review-Board Recommendation

This is the initial recommendation to review and either accept or revise before
PLAN-041 freezes public names:

- Python package name remains `dartpy`; first-use docs should teach
  `import dartpy as dart`.
- Python common path uses a small root facade: `dart.World(...)` is acceptable
  only as the exact same class object as `dart.simulation.World`.
- `dart.simulation` is the canonical Python owner for the full World API,
  simulation options, state objects, and advanced simulation workflows.
- Python root promotion is curated. Do not flatten implementation, backend,
  compute-stage, solver-registry, ECS, component, or migration-only symbols onto
  the root package.
- The initial root simulation allowance is `World`. Add `WorldOptions` or
  `WorldConfig` only if first-page docs need it, and keep the total root alias
  set explicitly allowlisted rather than derived from whole submodules.
- `dartpy.simulation_experimental` is not a DART 7 public owner. It is absent in
  the current generated stubs/runtime layout; if maintainers reintroduce it at
  all, it must be a short Python-level compatibility alias with warning,
  identity tests, stubs/docs coverage, and a removal condition.
- C++ first-use examples should use `dart::simulation::World` from
  `dart/simulation/world.hpp`. Do not add a top-level `dart::World` in the first
  promotion unless maintainers explicitly decide that the facade removes more
  friction than it adds.
- C++ public simulation headers live under `dart/simulation/` with snake_case
  names and final export macros. No promoted header should live under
  `experimental/`, `comps/`, `ecs/`, `detail/`, or an implementation-owned
  folder.
- The first promoted C++ concepts should be flat simulation concepts such as
  `World`, accepted options/configuration types, storage-neutral body/frame/
  multibody handles, contacts, query options, and curated diagnostics. Do not
  publish `body`, `frame`, `multibody`, `ecs`, or `comps` namespaces just
  because the staging tree has those folders.
- Official World is part of the default DART 7 package surface. Promotion is not
  complete while users need `dart-simulation-experimental` or public
  EnTT/Taskflow discovery to construct and step a basic World.
- Broad file moves wait until each moved surface has a row in this audit and
  package, docs, stubs, and API-boundary gates are green.

## Root Facade Rules

The root Python facade exists to make the common path easy, not to erase module
ownership.

| Rule               | Policy                                                                                                                                                             | Gate                                                                  |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------- |
| Symbol budget      | Promote only common user-facing classes/functions needed for first-page examples.                                                                                  | Runtime `__all__` and generated `__init__.pyi` match the decision.    |
| Identity           | Every root facade symbol is the same object as its canonical submodule owner.                                                                                      | `dart.World is dart.simulation.World` and alias `isinstance` tests.   |
| Owner modules      | Keep `simulation`, `io`, and `gui` importable; treat `math`, `collision`, and optimizer-style modules as advanced owner modules, not broad root-flattening inputs. | Import tests plus docs/stubs show canonical owner modules.            |
| Advanced features  | Keep advanced APIs in owner submodules unless explicitly approved for root promotion.                                                                              | Docs/stubs show owner submodule; no accidental root export.           |
| Migration symbols  | Keep compatibility aliases out of the root facade unless a maintainer approves a window.                                                                           | Deprecation warning, removal condition, and focused import tests.     |
| Implementation     | Never root-promote ECS, components, registries, backend/device types, or solver kernels.                                                                           | `check-api-boundaries` plus focused `dir(dart)`/`__all__` tests.      |
| Documentation path | First-use docs use `import dartpy as dart`; advanced docs introduce owner modules.                                                                                 | README/tutorial/API-doc examples do not mention experimental modules. |

## Required Check Surfaces

These checks are the enforcement slices for the promoted C++ name/target
surface:

- `pixi run check-dart7-promotion-surface` plus
  `pixi run check-simulation-public-headers`: verifies every promoted
  simulation header is allowlisted; recursive implementation installs are gone
  for promoted public API; forbidden include/signature tokens include EnTT,
  `entt::entity`, registries, `getRegistry()`, raw entity IDs,
  `detail::toRegistryEntity`, `EntityObject*`, `comps`, `ecs`, solver kernels,
  stage classes, backend adapters, Taskflow, CUDA/SYCL devices, streams, memory
  pools, and implementation-folder includes.
- `pixi run check-dartpy-import-layout`: verifies `dart.World is
dart.simulation.World` when runtime import is available, root stubs and
  `__all__` match the approved allowlist, runtime aliases in `sys.modules` match
  the accepted compatibility policy, generated stubs agree with runtime intent,
  and `simulation_experimental` is absent or a Python-level alias only.
- `pixi run check-dart7-promotion-package-contract`: verifies the staged CMake
  package prerequisites before the final package rename: the experimental module
  defaults on, diff/CUDA subfeatures remain opt-in, promoted headers are
  allowlist-installed rather than recursive, and EnTT/Taskflow/spdlog dependency
  discovery does not leak outside the static-link-only exception.
- `pixi run check-dart7-promotion-installed-package`: configures a minimal
  local install build, installs to a temporary prefix, compiles and runs a
  downstream CMake project against the staged promoted headers, and verifies
  internal ECS/detail/io/space headers are absent from the installed prefix.

The remaining missing check surface is the final-form installed-package smoke:
same positive/negative behavior, but retargeted to final C++ headers,
targets/components, and removed experimental package paths.

## Python Import Matrix

| Surface                                            | Recommended DART 7 outcome                                                                                  | Required evidence                                                                                    |
| -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `import dartpy as dart`                            | Canonical user import.                                                                                      | README/tutorial/example snippets use this path.                                                      |
| `dart.World`                                       | Root convenience alias to official simulation World.                                                        | `dart.World is dart.simulation.World`; included in `__all__` only if approved.                       |
| `dart.simulation.World`                            | Canonical owner class.                                                                                      | Single nanobind class registration; generated stubs/docs match runtime.                              |
| `dartpy.simulation.World`                          | Same canonical owner, using package name rather than alias.                                                 | Import smoke and wheel smoke.                                                                        |
| `dartpy.simulation_experimental`                   | Absent in the current DART 7 layout; Python-level alias only if maintainers reintroduce a migration window. | No second C++ binding; deprecation warning and removal condition if alias returns.                   |
| `from dartpy import simulation_experimental as sx` | Unsupported unless a maintainer approves a migration alias.                                                 | Import failure today; warning expectation, identity behavior, and sunset condition if alias returns. |
| `sx.World`                                         | Unsupported unless `sx` returns as a Python-level alias.                                                    | If returned, `sx.World is dart.simulation.World`; no duplicate `nb::class_`.                         |
| `sx.diff`                                          | Unsupported unless `sx` returns as a Python-level alias.                                                    | Current final owners are `dart.diff` and `dart.simulation.diff`; alias parity only if `sx` returns.  |
| `dart.simulation.diff`                             | Preferred final owner if differentiable World helpers stay simulation-specific.                             | Runtime import, generated stubs, docs, and torch-free import smoke.                                  |
| `sys.modules` aliases                              | Final modules plus documented migration aliases only.                                                       | Focused import-layout test for all accepted aliases.                                                 |
| Default build                                      | Public `dart.simulation.World` exists without staging build flags.                                          | Source/wheel smoke with the default DART 7 configuration.                                            |
| Typing artifacts                                   | Decide whether stubs are generated docs only or packaged typing artifacts.                                  | Wheel contents check for chosen `py.typed`/stub policy.                                              |

## C++ Header And Namespace Matrix

| Surface                                                 | Recommended DART 7 outcome                                       | Required evidence                                                                    |
| ------------------------------------------------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `dart::simulation::World`                               | Official C++ World owner after PLAN-041 name transaction.        | Classic World removed/quarantined; no second exported `dart::simulation::World`.     |
| `dart/simulation/world.hpp`                             | Canonical public include.                                        | Installs from allowlist; compiles from install tree; case-insensitive path check.    |
| `dart/simulation/fwd.hpp`                               | Public forward declarations for accepted lifecycle model.        | No stale `WorldPtr` unless shared ownership remains the accepted model.              |
| `WorldOptions` / `WorldConfig`                          | Accept one coherent public configuration story.                  | First-use examples and Python/C++ stubs/docs agree on constructor/config shape.      |
| `RigidBody`, `DeformableBody`, `Multibody`              | Candidate flat simulation concepts only if storage-neutral.      | No `entt::entity`, `getEntity`, ECS base, or registry access; focused tests.         |
| `Joint`, `Link`, `Frame`, `Contact`                     | Candidate flat simulation concepts only if lifecycle-safe.       | Lifetime/ownership tests; no staging folder namespace exposure.                      |
| `dart::World`                                           | Defer unless maintainers explicitly approve a C++ root facade.   | Decision record explains ABI/include/ownership impact if added.                      |
| `dart::simulation::experimental::*`                     | Not a promoted public namespace.                                 | No user docs/examples; package negative smoke rejects promoted use after cutover.    |
| `dart/simulation/comps/**`                              | Internal storage, never public.                                  | Not installed as promoted headers; no dartpy includes or public signatures.          |
| `dart/simulation/ecs/**`                                | Internal storage/access helpers, never public.                   | Public headers expose no entity IDs, registries, or ECS helper base classes.         |
| `dart/simulation/compute/**`                            | Advanced/backend-neutral compute API only after separate review. | No Taskflow/CUDA/device/stream/memory-pool leakage; benchmark-backed promotion gate. |
| `dart/simulation/compute` solver/kernels/stages/batches | Internal compute implementation.                                 | No installed headers; benchmark/focused tests only.                                  |
| `dart/simulation/diff/**`                               | Advanced differentiable owner to decide with PLAN-110.           | Final owner chosen; no framework/backend dependency in default public headers.       |
| `dart/simulation/space/**`                              | Public state wrappers or internal mapper implementation.         | No registry mapper public API; state docs if promoted.                               |
| `dart/simulation/io/**`                                 | IO-owned APIs or simulation adapters, not duplicate loaders.     | Loading front door remains `dart/io`; adapters hide implementation ownership.        |
| CMake target/component/export                           | Final package contract, not `simulation-experimental`.           | Installed package smoke, component scan, and private-dependency leak check.          |

## Repo-Wide Source Layout Audit

| Current owner                                           | Current role                                                       | DART 7 role                                                                                  | Proposed public name/source owner                                                                                                  | Migration action                                                               | Gate                                                                                 |
| ------------------------------------------------------- | ------------------------------------------------------------------ | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------ |
| `dart/common`                                           | Public utilities mixed with detail/foundation patterns.            | Foundation module with small public utility surface and larger internal support.             | Keep `dart::common`; keep internals in `detail/` or `internal/`.                                                                   | Keep and audit public-vs-internal rows.                                        | Boundary inventory covers any public detail exposure.                                |
| `dart/math`                                             | Public math foundations and advanced kernels.                      | Public foundation; first-use docs include only necessary math concepts.                      | Keep `dart::math`; advanced subfolders remain owner-specific.                                                                      | Keep; curate docs and Python root promotions.                                  | Stubs/docs root symbol budget; math tests.                                           |
| `dart/simd`                                             | Acceleration foundations and implementation-oriented variants.     | Advanced/internal acceleration owner, not first-use API.                                     | Keep `dart::simd` only for supported public SIMD concepts; otherwise internal.                                                     | Classify public SIMD vs private kernels.                                       | No backend/storage leak; SIMD tests for public rows.                                 |
| `dart/dynamics`                                         | DART 6 object model and public multibody concepts.                 | Compatibility/quarantine plus any surviving DART 7 multibody concepts.                       | Surviving concepts need explicit DART 7 names; legacy `Skeleton` path is not first-use.                                            | Decide remove, wrap, or quarantine per concept.                                | `release-6.*`/gz-physics support evidence for compatibility changes.                 |
| `dart/simulation`                                       | Official DART 7 World and simulation owner.                        | Official DART 7 simulation owner.                                                            | `dart::simulation::World`, `dart/simulation/world.hpp`.                                                                            | Keep promoted facade; prevent DART 6 World from returning on `main`.           | PLAN-041 gates, install smoke, C++/Python tests, boundary checks.                    |
| `dart/simulation/body`                                  | Public-looking body handles/options.                               | Candidate public concepts only if storage-neutral.                                           | `dart/simulation/*` concepts or owner-specific internal adapters.                                                                  | Promote selected wrappers; internalize storage-revealing handles.              | Header map proves no entity IDs, ECS bases, `comps/`, or registry access.            |
| `dart/simulation/frame`                                 | Frame handle concepts.                                             | Candidate public frame/state concepts after lifecycle review.                                | `dart/simulation/frame.hpp` or internal, depending on accepted World API.                                                          | Promote only storage-neutral value/handle API.                                 | Lifetime/ownership tests; no raw registry/entity exposure.                           |
| `dart/simulation/multibody`                             | Multibody handles/options.                                         | Candidate DART 7 multibody API if it replaces DART 6 dynamics concepts.                      | `dart/simulation/multibody.hpp` or a dedicated accepted owner.                                                                     | Decide relation to `dart/dynamics` before moving.                              | Cross-plan decision with PLAN-080/PLAN-041; examples and tests use final owner.      |
| `dart/simulation/space`                                 | State-space/vector mapper support.                                 | Advanced public or internal state-space owner.                                               | `dart/simulation/state.hpp`, `dart/simulation/space.hpp`, or internal.                                                             | Keep internal until API design proves user-facing need.                        | Docs explain state model; no mapper implementation leak.                             |
| `dart/simulation/compute`                               | Backend-neutral graph plus executors and kernels.                  | Advanced compute owner only after stable wrapper and benchmark gate.                         | `dart/simulation/compute/*`; private executors/kernels stay internal.                                                              | Split graph metadata from executors/kernels.                                   | `check-compute-backend-boundaries`, benchmarks, no Taskflow/CUDA public types.       |
| `dart/simulation/diff`                                  | Differentiable helper and rollout staging.                         | Final owner coordinated with PLAN-110; likely `dart.simulation.diff` in Python.              | `dart/simulation/diff/*` only after framework-neutral C++ API is stable.                                                           | Move with stubs/docs/runtime alias transaction.                                | Torch-free import smoke, generated stubs/docs, differentiable tests.                 |
| `dart/simulation/comps`                                 | ECS component storage.                                             | Internal only.                                                                               | Internal implementation owner.                                                                                                     | Keep hidden from install/docs/bindings.                                        | Negative header smoke; boundary checker rejects public includes/signatures.          |
| `dart/simulation/ecs`                                   | ECS access/helper templates.                                       | Internal only.                                                                               | Internal implementation owner.                                                                                                     | Keep hidden from install/docs/bindings.                                        | Boundary checker rejects EnTT, entity IDs, registries, helper-base inheritance.      |
| `dart/simulation/io`                                    | Loaders, serializers, model bridges.                               | `dart/io` front door plus simulation-owned adapters.                                         | Public loading in `dart/io`; private adapters under simulation/internal owners.                                                    | Move or wrap under IO decision; avoid duplicate public loaders.                | IO tests, parser docs, installed-header/package smoke.                               |
| `dart/io`                                               | Unified skeleton-loading front door.                               | Public loading owner.                                                                        | Keep `dart::io::readSkeleton` / `tryReadSkeleton` and Python `dart.io`; DART 6 `readWorld` parity lives on `release-6.*` branches. | Keep; absorb stable DART 7 model-loading APIs when ready.                      | IO tests, docs examples, package headers/targets.                                    |
| `dart/utils`                                            | Historical parser-specific utilities.                              | Compatibility/advanced parser owner behind `dart/io` common path.                            | Keep parser-specific APIs only as advanced/compatibility surface.                                                                  | Avoid teaching as first-use path.                                              | Deprecation policy, stubs/docs examples avoid `dart.utils` common path.              |
| `dart/collision`                                        | Collision concepts plus legacy backend facades.                    | Public collision-query concepts; backend facades compatibility only.                         | Keep `dart::collision`; backend details hidden behind native facade policy.                                                        | Keep public concepts; internalize backend plumbing.                            | Collision audits already in `pixi run lint`; boundary checks.                        |
| `dart/constraint`, `dart/lcpsolver`                     | Constraint APIs and low-level solver machinery.                    | Public constraint extension points; solver details internal or advanced.                     | Keep accepted user concepts; hide solver registries/kernels.                                                                       | Audit before exposing through simulation facade.                               | Constraint tests; API-boundary checks reject solver-registry leaks.                  |
| `dart/optimizer`                                        | Optimization wrappers and optional dependency facades.             | Advanced public module.                                                                      | Keep explicit `dart::optimizer`; no root/common-path promotion by default.                                                         | Keep; document as advanced.                                                    | Optional dependency/package checks.                                                  |
| `dart/gui`, `dart/sensor`                               | Runtime visualization/sensor concepts with backend implementation. | Public concepts remain backend-hidden.                                                       | Keep `dart::gui` / `dart::sensor`; simulation composes, does not duplicate.                                                        | Keep public owner; adapt World integration after promotion.                    | GUI/sensor tests; no Filament/GLFW/backend leakage.                                  |
| `python/dartpy`                                         | Root flattening, legacy module wrappers, C++ extension submodules. | Public Python package and curated root facade.                                               | Keep `dartpy`; teach `import dartpy as dart`; canonical submodules remain.                                                         | Update `_layout.py`, `_naming.py`, `__all__`, imports together.                | Runtime import-layout tests, generated stubs/docs, wheel smoke.                      |
| `python/dartpy/simulation`                              | C++ binding source for the promoted `dartpy.simulation` module.    | Public Python simulation owner.                                                              | `dartpy.simulation`; `dartpy.World` is the approved root convenience alias.                                                        | Keep runtime/stubs/docs/layout synchronized; no runtime alias unless approved. | `check-dartpy-import-layout`, no duplicate nanobind class registration, stubs/docs.  |
| `python/stubs/dartpy`, generated API docs               | Generated public Python surface.                                   | Must match runtime topology exactly.                                                         | Final pages for root facade and owner modules; no stale experimental docs.                                                         | Regenerate with same PR as runtime import changes.                             | `generate-stubs`, `api-docs-py`, docs policy, stub diff review.                      |
| CMake components and installed package exports          | Core `dart` plus `dart-simulation` component.                      | Official World usable from default DART 7 package; optional accelerators private.            | Final target/component/export decision before source move.                                                                         | Keep experimental component retired after promotion.                           | Source-build/install smokes, reduced-build checks, target/component negative smokes. |
| `cmake/DARTConfigVersion.cmake.in` and package metadata | DART 7 package versioning still has DART 6 migration concerns.     | Release-owner decision aligning DART 7 clean break with any time-boxed compatibility canary. | Final DART 7 package config/version policy.                                                                                        | Update release docs/package metadata together.                                 | Changelog/migration notes, `release-6.*` support window, gz-physics lane policy.     |
| Wheel build/verify scripts                              | Wheel defaults may differ from Pixi dev configuration.             | Public World present in release wheels without experimental flags.                           | Wheel CMake args/defaults and verify/test assertions.                                                                              | Update wheel build and wheel smoke with import-layout assertions.              | Build/repair/verify/test wheels; feature-off World import and step smoke.            |

## Package And Release Gates

Before PLAN-041 claims official World promotion:

- Default source builds import and exercise the final World path from C++ and
  Python without staging build flags.
- Installed C++ quickstart uses only final includes, final package targets, and
  `find_package(DART 7.0.0 REQUIRED CONFIG)`; it constructs and steps one
  official World.
- Negative installed-package smokes reject forbidden experimental headers,
  recursive staging-tree headers, and obsolete `simulation-experimental`
  package components once promotion is claimed.
- Installed component/config files expose no private include directories,
  EnTT/Taskflow/CUDA/SYCL runtime requirements, private implementation targets,
  or staging-build-option requirement for the official World. The static
  prerequisite is guarded by `check-dart7-promotion-package-contract` and the
  staged local-install smoke is guarded by
  `check-dart7-promotion-installed-package`; the final-form package smoke still
  needs the accepted final headers/targets.
- Wheel verify/test asserts `dart.World is dart.simulation.World`, absent
  `simulation_experimental` unless a maintainer approves an alias, `__all__`,
  `sys.modules`, stubs/docs parity, and `dart.diff` /
  `dart.simulation.diff` behavior for the chosen final home.
- Platform validation covers macOS or another case-insensitive filesystem for
  `world.hpp` and generated compatibility headers; Windows package configure is
  required if target exports change.
- Release metadata, changelog/migration notes, `release-6.*` support window, and
  gz-physics lane policy agree before DART 7 publication.

## Implementation Gate Set

Implementation PRs derived from this audit select by touched scope, but a
promotion PR must include:

- `pixi run check-lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-simulation-quick`
- `pixi run test-py`
- `pixi run generate-stubs`
- `pixi run api-docs-py`
- `pixi run check-api-boundaries`
- the public-header allowlist check
- the package-contract check
- the installed-package smoke
- the dartpy import-layout check
- installed C++ positive and negative package smokes
- wheel build/verify/test import smokes
- feature-off public World smoke

## First Follow-Up Work Items

1. Use
   [`post-promotion-source-layout-decision.md`](post-promotion-source-layout-decision.md)
   as the current PR #2932 folder decision: the promoted `dart/simulation` tree
   is accepted as a guarded transition layout, and broad physical moves wait for
   owner-specific follow-up rows and gates.
2. Keep the promoted experimental-header allowlist, strict promotion audit,
   public-header smoke, package-contract check, and installed-package smoke
   green while preparing the C++ name/target transaction.
3. Keep `check-dartpy-import-layout`, generated stubs, and Python tests green
   for `dart.World`, `dart.simulation.World`, absent `simulation_experimental`,
   `dart.diff` / `dart.simulation.diff`, `__all__`, and `sys.modules`.
4. Retarget the installed-package C++ smoke to
   `#include <dart/simulation/world.hpp>` and negative smokes for forbidden
   experimental headers/targets after the final C++ header/target decision
   lands.
5. Update PLAN-041 readiness audit to consume the accepted rows for the World
   transaction only, leaving unrelated folder renovations to later PLAN-042
   batches.
