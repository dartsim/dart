# PLAN-042 API And Source Layout Audit

- Owner plan:
  [`../042-dart7-public-api-and-source-layout.md`](../042-dart7-public-api-and-source-layout.md)
- Operating state: `PLAN-042` in [`../dashboard.md`](../dashboard.md)
- Purpose: classify the current C++ folders, Python modules, generated surfaces,
  and package exports before DART 7 freezes the official World API or performs
  broad source moves.

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
- `dartpy.simulation_experimental` is not a DART 7 public owner. If retained at
  all, it is a short Python-level compatibility alias with warning, identity
  tests, stubs/docs coverage, and a removal condition.
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
  complete while users need `DART_BUILD_SIMULATION_EXPERIMENTAL`,
  `dart-simulation-experimental`, or public EnTT/Taskflow discovery to construct
  and step a basic World.
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

## Required New Check Surfaces

Add these checks before the first implementation PR that freezes final names:

- `check-simulation-public-header-allowlist`: verifies every promoted
  simulation header is allowlisted; recursive `experimental/` installs are gone
  for promoted public API; forbidden include/signature tokens include EnTT,
  `entt::entity`, registries, `getRegistry()`, `EntityObject*`, `comps`, `ecs`,
  solver kernels, stage classes, backend adapters, Taskflow, CUDA/SYCL devices,
  streams, memory pools, and implementation-folder includes.
- `check-dartpy-import-layout`: verifies `dart.World is
dart.simulation.World`, root `__all__` matches the approved allowlist, runtime
  aliases in `sys.modules` match the accepted compatibility policy, generated
  stubs/docs agree with runtime, and `simulation_experimental` is absent or a
  Python-level alias only.

These checks may be dedicated scripts or folded into existing API-boundary
checks, but PLAN-042 implementation PRs must produce the equivalent evidence.

## Python Import Matrix

| Surface                                            | Recommended DART 7 outcome                                                      | Required evidence                                                                  |
| -------------------------------------------------- | ------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `import dartpy as dart`                            | Canonical user import.                                                          | README/tutorial/example snippets use this path.                                    |
| `dart.World`                                       | Root convenience alias to official simulation World.                            | `dart.World is dart.simulation.World`; included in `__all__` only if approved.     |
| `dart.simulation.World`                            | Canonical owner class.                                                          | Single nanobind class registration; generated stubs/docs match runtime.            |
| `dartpy.simulation.World`                          | Same canonical owner, using package name rather than alias.                     | Import smoke and wheel smoke.                                                      |
| `dartpy.simulation_experimental`                   | Removed or Python-level alias only.                                             | No second C++ binding; deprecation warning and removal condition if alias remains. |
| `from dartpy import simulation_experimental as sx` | Compatibility-only if retained.                                                 | Import smoke, warning expectation, identity behavior, and sunset condition.        |
| `sx.World`                                         | Compatibility alias only if `sx` remains.                                       | `sx.World is dart.simulation.World`; no duplicate `nb::class_`.                    |
| `sx.diff`                                          | Temporary alias to the chosen final differentiable owner.                       | `sx.diff.timestep` lazy torch behavior preserved; stubs/docs final-home decision.  |
| `dart.simulation.diff`                             | Preferred final owner if differentiable World helpers stay simulation-specific. | Runtime import, generated stubs, docs, and torch-free import smoke.                |
| `sys.modules` aliases                              | Final modules plus documented migration aliases only.                           | Focused import-layout test for all accepted aliases.                               |
| Feature-off build                                  | Public `dart.simulation.World` survives without experimental build flags.       | Feature-off source/wheel smoke with `DART_BUILD_SIMULATION_EXPERIMENTAL=OFF`.      |
| Typing artifacts                                   | Decide whether stubs are generated docs only or packaged typing artifacts.      | Wheel contents check for chosen `py.typed`/stub policy.                            |

## C++ Header And Namespace Matrix

| Surface                                                              | Recommended DART 7 outcome                                       | Required evidence                                                                    |
| -------------------------------------------------------------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `dart::simulation::World`                                            | Official C++ World owner after PLAN-041 name transaction.        | Classic World removed/quarantined; no second exported `dart::simulation::World`.     |
| `dart/simulation/world.hpp`                                          | Canonical public include.                                        | Installs from allowlist; compiles from install tree; case-insensitive path check.    |
| `dart/simulation/fwd.hpp`                                            | Public forward declarations for accepted lifecycle model.        | No stale `WorldPtr` unless shared ownership remains the accepted model.              |
| `WorldOptions` / `WorldConfig`                                       | Accept one coherent public configuration story.                  | First-use examples and Python/C++ stubs/docs agree on constructor/config shape.      |
| `RigidBody`, `DeformableBody`, `Multibody`                           | Candidate flat simulation concepts only if storage-neutral.      | No `entt::entity`, `getEntity`, ECS base, or registry access; focused tests.         |
| `Joint`, `Link`, `Frame`, `Contact`                                  | Candidate flat simulation concepts only if lifecycle-safe.       | Lifetime/ownership tests; no staging folder namespace exposure.                      |
| `dart::World`                                                        | Defer unless maintainers explicitly approve a C++ root facade.   | Decision record explains ABI/include/ownership impact if added.                      |
| `dart::simulation::experimental::*`                                  | Not a promoted public namespace.                                 | No user docs/examples; package negative smoke rejects promoted use after cutover.    |
| `dart/simulation/experimental/comps/**`                              | Internal storage, never public.                                  | Not installed as promoted headers; no dartpy includes or public signatures.          |
| `dart/simulation/experimental/ecs/**`                                | Internal storage/access helpers, never public.                   | Public headers expose no entity IDs, registries, or ECS helper base classes.         |
| `dart/simulation/experimental/compute/**`                            | Advanced/backend-neutral compute API only after separate review. | No Taskflow/CUDA/device/stream/memory-pool leakage; benchmark-backed promotion gate. |
| `dart/simulation/experimental/compute` solver/kernels/stages/batches | Internal compute implementation.                                 | No installed headers; benchmark/focused tests only.                                  |
| `dart/simulation/experimental/diff/**`                               | Advanced differentiable owner to decide with PLAN-110.           | Final owner chosen; no framework/backend dependency in default public headers.       |
| `dart/simulation/experimental/space/**`                              | Public state wrappers or internal mapper implementation.         | No registry mapper public API; state docs if promoted.                               |
| `dart/simulation/experimental/io/**`                                 | IO-owned APIs or simulation adapters, not duplicate loaders.     | Loading front door remains `dart/io`; adapters hide implementation ownership.        |
| CMake target/component/export                                        | Final package contract, not `simulation-experimental`.           | Installed package smoke, component scan, and private-dependency leak check.          |

## Repo-Wide Source Layout Audit

| Current owner                                           | Current role                                                       | DART 7 role                                                                                  | Proposed public name/source owner                                                       | Migration action                                                  | Gate                                                                                 |
| ------------------------------------------------------- | ------------------------------------------------------------------ | -------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `dart/common`                                           | Public utilities mixed with detail/foundation patterns.            | Foundation module with small public utility surface and larger internal support.             | Keep `dart::common`; keep internals in `detail/` or `internal/`.                        | Keep and audit public-vs-internal rows.                           | Boundary inventory covers any public detail exposure.                                |
| `dart/math`                                             | Public math foundations and advanced kernels.                      | Public foundation; first-use docs include only necessary math concepts.                      | Keep `dart::math`; advanced subfolders remain owner-specific.                           | Keep; curate docs and Python root promotions.                     | Stubs/docs root symbol budget; math tests.                                           |
| `dart/simd`                                             | Acceleration foundations and implementation-oriented variants.     | Advanced/internal acceleration owner, not first-use API.                                     | Keep `dart::simd` only for supported public SIMD concepts; otherwise internal.          | Classify public SIMD vs private kernels.                          | No backend/storage leak; SIMD tests for public rows.                                 |
| `dart/dynamics`                                         | DART 6 object model and public multibody concepts.                 | Compatibility/quarantine plus any surviving DART 7 multibody concepts.                       | Surviving concepts need explicit DART 7 names; legacy `Skeleton` path is not first-use. | Decide remove, wrap, or quarantine per concept.                   | DART 6.17/gz-physics support evidence for compatibility changes.                     |
| `dart/simulation`                                       | Classic World owns official name today.                            | Official DART 7 simulation owner after PLAN-041 transaction.                                 | `dart::simulation::World`, `dart/simulation/world.hpp`.                                 | Replace/quarantine classic World; promote ECS-backed facade.      | PLAN-041 gates, install smoke, C++/Python tests, boundary checks.                    |
| `dart/simulation/experimental/body`                     | Staging public-looking body handles/options.                       | Candidate public concepts only if storage-neutral.                                           | Likely `dart/simulation/*` concepts or owner-specific internal adapters.                | Promote selected wrappers; internalize storage-revealing handles. | Header map proves no entity IDs, ECS bases, `comps/`, or registry access.            |
| `dart/simulation/experimental/frame`                    | Staging frame handle concepts.                                     | Candidate public frame/state concepts after lifecycle review.                                | `dart/simulation/frame.hpp` or internal, depending on accepted World API.               | Promote only storage-neutral value/handle API.                    | Lifetime/ownership tests; no raw registry/entity exposure.                           |
| `dart/simulation/experimental/multibody`                | Staging multibody handles/options.                                 | Candidate DART 7 multibody API if it replaces DART 6 dynamics concepts.                      | `dart/simulation/multibody.hpp` or a dedicated accepted owner.                          | Decide relation to `dart/dynamics` before moving.                 | Cross-plan decision with PLAN-080/PLAN-041; examples and tests use final owner.      |
| `dart/simulation/experimental/space`                    | State-space/vector mapper support.                                 | Advanced public or internal state-space owner.                                               | `dart/simulation/state.hpp`, `dart/simulation/space.hpp`, or internal.                  | Keep internal until API design proves user-facing need.           | Docs explain state model; no mapper implementation leak.                             |
| `dart/simulation/experimental/compute`                  | Backend-neutral graph plus executors and kernels.                  | Advanced compute owner only after stable wrapper and benchmark gate.                         | Maybe `dart/simulation/compute/*`; private executors/kernels stay internal.             | Split graph metadata from executors/kernels.                      | `check-compute-backend-boundaries`, benchmarks, no Taskflow/CUDA public types.       |
| `dart/simulation/experimental/diff`                     | Differentiable helper and rollout staging.                         | Final owner coordinated with PLAN-110; likely `dart.simulation.diff` in Python.              | `dart/simulation/diff/*` only after framework-neutral C++ API is stable.                | Move with stubs/docs/runtime alias transaction.                   | Torch-free import smoke, generated stubs/docs, differentiable tests.                 |
| `dart/simulation/experimental/comps`                    | ECS component storage.                                             | Internal only.                                                                               | Internal implementation owner.                                                          | Stop recursive public install; hide from docs/bindings.           | Negative header smoke; boundary checker rejects public includes/signatures.          |
| `dart/simulation/experimental/ecs`                      | ECS access/helper templates.                                       | Internal only.                                                                               | Internal implementation owner.                                                          | Stop recursive public install; hide inheritance/accessors.        | Boundary checker rejects EnTT, entity IDs, registries, helper-base inheritance.      |
| `dart/simulation/experimental/io`                       | Experimental loaders, serializers, model bridges.                  | `dart/io` front door plus simulation-owned adapters.                                         | Public loading in `dart/io`; private adapters under simulation/internal owners.         | Move or wrap under IO decision; avoid duplicate public loaders.   | IO tests, parser docs, installed-header/package smoke.                               |
| `dart/io`                                               | Unified model-loading front door.                                  | Public loading owner.                                                                        | Keep `dart::io::readWorld` / `readSkeleton` and Python `dart.io`.                       | Keep; absorb stable experimental loading APIs when ready.         | IO tests, docs examples, package headers/targets.                                    |
| `dart/utils`                                            | Historical parser-specific utilities.                              | Compatibility/advanced parser owner behind `dart/io` common path.                            | Keep parser-specific APIs only as advanced/compatibility surface.                       | Avoid teaching as first-use path.                                 | Deprecation policy, stubs/docs examples avoid `dart.utils` common path.              |
| `dart/collision`                                        | Collision concepts plus legacy backend facades.                    | Public collision-query concepts; backend facades compatibility only.                         | Keep `dart::collision`; backend details hidden behind native facade policy.             | Keep public concepts; internalize backend plumbing.               | Collision audits already in `pixi run lint`; boundary checks.                        |
| `dart/constraint`, `dart/lcpsolver`                     | Constraint APIs and low-level solver machinery.                    | Public constraint extension points; solver details internal or advanced.                     | Keep accepted user concepts; hide solver registries/kernels.                            | Audit before exposing through simulation facade.                  | Constraint tests; API-boundary checks reject solver-registry leaks.                  |
| `dart/optimizer`                                        | Optimization wrappers and optional dependency facades.             | Advanced public module.                                                                      | Keep explicit `dart::optimizer`; no root/common-path promotion by default.              | Keep; document as advanced.                                       | Optional dependency/package checks.                                                  |
| `dart/gui`, `dart/sensor`                               | Runtime visualization/sensor concepts with backend implementation. | Public concepts remain backend-hidden.                                                       | Keep `dart::gui` / `dart::sensor`; simulation composes, does not duplicate.             | Keep public owner; adapt World integration after promotion.       | GUI/sensor tests; no Filament/GLFW/backend leakage.                                  |
| `python/dartpy`                                         | Root flattening, legacy module wrappers, C++ extension submodules. | Public Python package and curated root facade.                                               | Keep `dartpy`; teach `import dartpy as dart`; canonical submodules remain.              | Update `_layout.py`, `_naming.py`, `__all__`, imports together.   | Runtime import-layout tests, generated stubs/docs, wheel smoke.                      |
| `python/dartpy/simulation_experimental`                 | Separate C++ binding module for staging World.                     | Removed or Python-level alias only.                                                          | No final C++ owner; forward to `dartpy.simulation` if alias remains.                    | Rebind once, then remove/alias with warnings.                     | No duplicate nanobind class registration; identity and `isinstance` tests.           |
| `python/stubs/dartpy`, generated API docs               | Generated public Python surface.                                   | Must match runtime topology exactly.                                                         | Final pages for root facade and owner modules; no stale experimental docs.              | Regenerate with same PR as runtime import changes.                | `generate-stubs`, `api-docs-py`, docs policy, stub diff review.                      |
| CMake components and installed package exports          | Core `dart` plus optional `simulation-experimental` component.     | Official World usable from default DART 7 package; optional accelerators private.            | Final target/component/export decision before source move.                              | Remove or quarantine experimental component after promotion.      | Source-build/install smokes, reduced-build checks, target/component negative smokes. |
| `cmake/DARTConfigVersion.cmake.in` and package metadata | DART 7 package versioning still has DART 6 migration concerns.     | Release-owner decision aligning DART 7 clean break with any time-boxed compatibility canary. | Final DART 7 package config/version policy.                                             | Update release docs/package metadata together.                    | Changelog/migration notes, DART 6.17 support window, gz-physics lane policy.         |
| Wheel build/verify scripts                              | Wheel defaults may differ from Pixi dev configuration.             | Public World present in release wheels without experimental flags.                           | Wheel CMake args/defaults and verify/test assertions.                                   | Update wheel build and wheel smoke with import-layout assertions. | Build/repair/verify/test wheels; feature-off World import and step smoke.            |

## Package And Release Gates

Before PLAN-041 claims official World promotion:

- Reduced source build with `DART_BUILD_SIMULATION_EXPERIMENTAL=OFF` still
  builds, imports, and exercises the final World path from C++ and Python.
- Installed C++ quickstart uses only final includes, final package targets, and
  `find_package(DART 7.0.0 REQUIRED CONFIG)`; it constructs and steps one
  official World.
- Negative installed-package smokes reject forbidden experimental headers,
  recursive staging-tree headers, and obsolete `simulation-experimental`
  package components once promotion is claimed.
- Installed component/config files expose no private include directories,
  EnTT/Taskflow/CUDA/SYCL runtime requirements, private implementation targets,
  or `DART_BUILD_SIMULATION_EXPERIMENTAL` requirement for the official World.
- Wheel verify/test asserts `dart.World is dart.simulation.World`, the accepted
  `simulation_experimental` policy, `__all__`, `sys.modules`, stubs/docs parity,
  and `sx.diff` or `dart.simulation.diff` behavior for the chosen final home.
- Platform validation covers macOS or another case-insensitive filesystem for
  `world.hpp` and generated compatibility headers; Windows package configure is
  required if target exports change.
- Release metadata, changelog/migration notes, DART 6.17 support window, and
  gz-physics lane policy agree before DART 7 publication.

## Implementation Gate Set

Implementation PRs derived from this audit select by touched scope, but a
promotion PR must include:

- `pixi run check-lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-simulation-experimental` until the target/test name is renamed
- `pixi run test-py`
- `pixi run generate-stubs`
- `pixi run api-docs-py`
- `pixi run check-api-boundaries`
- `pixi run check-api-boundary-inventory`
- the public-header allowlist check
- the dartpy import-layout check
- installed C++ positive and negative package smokes
- wheel build/verify/test import smokes
- feature-off public World smoke

## First Follow-Up Work Items

1. Record a maintainer decision on Python root facade scope and C++ root facade
   deferral.
2. Add a generated or checked promoted-simulation header allowlist before moving
   any `simulation/experimental` file.
3. Add focused Python import-layout tests for `dart.World`,
   `dart.simulation.World`, `simulation_experimental`, `sx.diff`, `__all__`, and
   `sys.modules`.
4. Add installed-package C++ smokes for `#include <dart/simulation/world.hpp>`
   and negative smokes for forbidden experimental headers/targets.
5. Update PLAN-041 readiness audit to consume the accepted rows for the World
   transaction only, leaving unrelated folder renovations to later PLAN-042
   batches.
