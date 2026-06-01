# PLAN-041: Official Simulation API Promotion

- Operating state: `PLAN-041` in [`dashboard.md`](dashboard.md)
- Outcome: the ECS-backed simulation world becomes the official DART 7 C++ and
  dartpy simulation API, with the experimental namespace/module removed from the
  promoted public contract and the source tree reorganized only after the stable
  facade is in place. DART 6 API compatibility remains on the DART 6.16 support
  lane; main moves as a DART 7 clean break.
- Current evidence: the experimental world already owns the DART 7 simulation
  direction, but its current public shape still exposes experimental paths,
  implementation-oriented headers, build-option seams, and a Python module split
  that cannot be treated as the final API without a deliberate promotion pass.

## Direction

Use a staged DART 7 promotion, not a DART 8 middle step.

The fastest maintainable path is:

1. Make the official API contract explicit.
2. Promote a small stable facade.
3. Move consumers to that facade.
4. Delete or quarantine the legacy DART 6 simulation surface on main.
5. Perform physical source-tree moves after the facade is green.

Do not start with a broad `git mv` of `experimental/`. Mechanical moves create
large conflicts and do not by themselves solve namespace, ABI, Python module, or
package-export questions. The source-tree restructuring is important, but it is
late-stage cleanup after official names and gates are stable.

## Compatibility Policy

- `main` targets DART 7 and should not carry the DART 6 public API as a long-term
  compatibility layer.
- gz-physics and Gazebo compatibility work remains important, but compatibility
  fixes for the existing DART 6 API belong on `release-6.16` unless a maintainer
  explicitly scopes a main-branch migration canary.
- DART 8 is reserved for debt introduced during the DART 7 line. It is not the
  default place to remove DART 6 simulation APIs or the experimental-world name.
- Temporary aliases are acceptable only as migration aids with owner docs,
  tests, and removal criteria.

## Specialist Review Inputs

The plan incorporates three focused review perspectives before implementation:

- **C++ API**: final public headers must not expose ECS storage, EnTT, component
  namespaces, solver registries, backend types, or implementation folders. The
  current `dart::simulation::World` name is occupied by the classic API, so the
  collision must be resolved before the ECS-backed world can own the official
  namespace. That resolution must be an explicit transaction: either introduce a
  non-colliding temporary facade while consumers migrate, or atomically replace
  the classic definition with the ECS-backed facade in one PR. Any quarantine may
  preserve the old implementation only under an explicit legacy or internal
  namespace/target; it must not keep exporting `dart::simulation::World`.
- **Python bindings**: the likely promoted import path is `dartpy.simulation`,
  with `dartpy.simulation_experimental` reduced to a short
  compatibility alias only if needed. Avoid duplicate nanobind class
  registration. Do not repoint top-level `dartpy.World` without an explicit
  maintainer decision because that name currently implies the legacy simulation
  module. The promotion must decide whether `dartpy.World` is removed,
  deprecated, or identical to `dartpy.simulation.World`, and gate that decision
  with tests, stubs, generated docs, and wheel import smokes.
- **Build, package, and downstream**: decide whether the promoted simulation API
  is a core component or a separately discoverable CMake component before
  reshaping install/export rules. The official baseline API must be present in
  default builds before promotion is claimed; it cannot remain hidden behind the
  old `DART_BUILD_SIMULATION_EXPERIMENTAL` option or require a
  `simulation-experimental` target to link. Keep reduced-build and package smoke
  tests in the gate while the old option still exists.

## Workstreams

1. **Promotion contract and readiness audit** - freeze the initial supported
   public subset, record the headers/modules to promote, list public-looking
   internals to hide, and map parity evidence that blocks the promotion claim.
   This step edits docs and boundary inventories; it does not move source files.
2. **Build and package shape** - choose the final target/component/export-macro
   shape before the first promoted API PR. Make the official baseline API
   non-optional in default builds, update install/export rules, and add package
   smokes that prove promoted headers and Python modules are available without
   experimental build flags.
3. **Promoted API boundary enforcement** - update boundary inventories and
   checks so promoted C++ headers fail if they expose EnTT, ECS storage,
   component namespaces, solver registries, backend types, implementation
   folders, or direct registry access. Add negative installed-package smokes for
   forbidden headers and obsolete experimental targets.
4. **World name-collision transaction design** - choose how to clear the classic
   `dart::simulation::World` collision before implementation. Either use a
   temporary non-colliding facade while consumers migrate, or replace the classic
   `World` definition and introduce the ECS-backed facade atomically. A
   quarantine plan must not continue exporting `dart::simulation::World`.
5. **C++ official facade** - introduce the promoted C++ API under the chosen
   namespace and public include path. Use wrappers or opaque implementation
   ownership where needed so public headers do not expose ECS internals. This is
   a semantic facade step; broad physical `git mv` work waits until later.
6. **Python official facade** - bind the promoted API under `dartpy.simulation`,
   decide the top-level `dartpy.World` outcome, generate stubs and docs for the
   final path, and make `dartpy.simulation_experimental` a short compatibility
   alias only if the release path needs it. Cover `sx.diff` and any alias window
   in the migration matrix.
7. **Consumer migration** - port examples, tutorials, tests, benchmarks, stubs,
   package quickstarts, and readthedocs snippets to the official path. Remove
   experimental imports from user-facing docs once the replacement is covered.
8. **Legacy API removal or quarantine completion** - after the replacement path
   is usable, remove the DART 6 simulation surface from the DART 7 public
   contract on main, or quarantine it behind explicitly non-default migration
   support that no longer owns `dart::simulation::World`. This avoids shipping
   two official worlds.
9. **Physical source-tree restructuring** - after the official facade is green,
   split mechanical moves into reviewable PRs:
   - public C++ headers under the final `dart/simulation/` layout;
   - implementation-only files under an internal/detail layout;
   - Python binding files under the final `python/dartpy/simulation/` layout;
   - generated docs, generated compatibility headers, and boundary inventories
     updated in the same move.
10. **Alias and option cleanup** - remove experimental aliases, obsolete build
    options, stale path filters, old package components, and remaining migration
    shims once the official path is the only supported DART 7 path.

## High-Risk Surfaces

Review these before implementation PRs:

- `dart/simulation/world.hpp` and the classic `dart::simulation::World`
  namespace collision.
- `dart/simulation/experimental/world.hpp`, especially exposed registry or ECS
  access.
- `dart/simulation/experimental/ecs/` and `comps/` headers that look public only
  because they are installed today.
- `dart/simulation/experimental/*_loader.*`, frame/body/joint handles, and stage
  APIs that may currently reveal implementation ownership.
- `dart/simulation/experimental/CMakeLists.txt`, root CMake options, package
  exports, and workflow path filters.
- `dart/simulation/CMakeLists.txt` generated component headers and PascalCase
  compatibility headers, especially `World.hpp`/`world.hpp` case-insensitive
  filesystem collisions.
- `python/dartpy/dartpy.cpp`, `python/dartpy/simulation/`,
  `python/dartpy/simulation_experimental/`, stub generation, and Python API
  boundary checks.
- `scripts/check_api_boundaries.py`, `scripts/generate_api_boundary_inventory.py`,
  `scripts/generate_stubs.py`, and generated API docs that currently know about
  experimental module names.

## Acceptance Criteria

- The official C++ path builds from default source builds and from an installed
  package using only promoted headers, final package targets, and final export
  macros.
- The official Python path imports as `dartpy.simulation`, has generated stubs
  and generated docs, and does not duplicate nanobind class identities.
- The top-level `dartpy.World` decision is explicit: removed, deprecated, or
  identical to `dartpy.simulation.World`, with tests, stubs, docs, and wheel
  import smokes covering the chosen behavior.
- If migration aliases exist, the compatibility matrix covers
  `import dartpy.simulation`, `import dartpy.simulation_experimental`,
  `from dartpy import simulation_experimental as sx`, `sx.diff`, and
  `isinstance`/identity behavior across aliases.
- Public API boundary checks reject ECS storage, EnTT, component types, solver
  registries, backend types, direct registry access, implementation folders, and
  promoted headers that include internal-only paths.
- Installed-package negative smokes reject forbidden experimental headers and
  obsolete `simulation-experimental` package targets once promotion is claimed.
- In-repo examples, tutorials, tests, and docs use the official path unless they
  are explicitly testing a migration alias.
- DART 6/gz-physics compatibility expectations are documented on the DART 6.16
  lane, including the Gazebo branch/version matrix, support window, and sunset
  date or trigger before main removes legacy API surfaces.
- Source-tree moves happen after facade promotion and are isolated enough that
  active solver and loader branches can merge latest main without broad manual
  conflict surgery. Preflight broad moves with `git merge-tree` against active
  PR heads and handle generated compatibility headers in the same move.

## Verification Gates

For the planning PR:

- `pixi run lint-md`
- `pixi run check-lint-md`
- `pixi run check-docs-policy`
- `pixi run check-lint-spell`
- `pixi run lint`

For implementation PRs, select by touched scope:

- `pixi run check-api-boundaries`
- `pixi run check-api-boundary-inventory`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-simulation-experimental`
- `pixi run test-py`
- `pixi run test-all`
- `pixi run -e cuda test-all` when CUDA, compute, or full-release promotion
  confidence is needed
- `pixi run generate-stubs` and `pixi run api-docs-py` when Python module names,
  `dartpy.World`, or generated docs change
- installed-package C++ and wheel import smokes for package/export changes
- negative installed-package smokes for forbidden promoted-header dependencies
  and obsolete experimental targets
- reduced-build checks while `DART_BUILD_SIMULATION_EXPERIMENTAL` exists

## Revision Triggers

- A maintainer chooses a different final C++ namespace, Python import path, or
  CMake component shape.
- Parity gates expose a blocker that prevents official promotion in DART 7.
- gz-physics or Gazebo compatibility requires a DART 6.16 support-lane update.
- A physical move PR conflicts heavily with active solver/model-loading work and
  needs to be split or delayed.
