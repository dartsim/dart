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
  namespace. A stable facade is safer than exporting the current experimental
  class layout as-is.
- **Python bindings**: the likely promoted import path is `dartpy.simulation`,
  with `dartpy.simulation_experimental` reduced to a short
  compatibility alias only if needed. Avoid duplicate nanobind class
  registration. Do not repoint top-level `dartpy.World` without an explicit
  maintainer decision because that name currently implies the legacy simulation
  module.
- **Build, package, and downstream**: decide whether the promoted simulation API
  is a core component or a separately discoverable CMake component before
  reshaping install/export rules. Keep reduced-build and package smoke tests in
  the gate while the old `DART_BUILD_SIMULATION_EXPERIMENTAL` option still
  exists.

## Workstreams

1. **Promotion contract and readiness audit** - freeze the initial supported
   public subset, record the headers/modules to promote, list public-looking
   internals to hide, and map parity evidence that blocks the promotion claim.
   This step edits docs and boundary inventories; it does not move source files.
2. **Build and package facade** - choose the promoted CMake/package shape,
   update install/export rules, and add an installed-package smoke that includes
   only promoted headers. Keep old optional-build behavior covered until the
   option is removed.
3. **Classic API removal or quarantine** - remove the DART 6 simulation surface
   from the DART 7 public contract on main, or quarantine it behind explicitly
   non-default migration support. This clears the `dart::simulation::World`
   collision and avoids shipping two official worlds.
4. **C++ official facade** - introduce the promoted C++ API under the final
   official namespace and header layout. Use wrappers or opaque implementation
   ownership where needed so the ABI and public headers do not expose ECS
   internals.
5. **Python official facade** - bind the promoted API under `dartpy.simulation`,
   generate stubs and docs for that path, and make
   `dartpy.simulation_experimental` a short compatibility alias only if the
   release path needs it.
6. **Consumer migration** - port examples, tutorials, tests, benchmarks, stubs,
   package quickstarts, and readthedocs snippets to the official path. Remove
   experimental imports from user-facing docs once the replacement is covered.
7. **Physical source-tree restructuring** - after the official facade is green,
   split mechanical moves into reviewable PRs:
   - public C++ headers under the final `dart/simulation/` layout;
   - implementation-only files under an internal/detail layout;
   - Python binding files under the final `python/dartpy/simulation/` layout;
   - generated docs and boundary inventories updated in the same move.
8. **Alias and option cleanup** - remove experimental aliases, obsolete build
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
- `python/dartpy/dartpy.cpp`, `python/dartpy/simulation/`,
  `python/dartpy/simulation_experimental/`, stub generation, and Python API
  boundary checks.

## Acceptance Criteria

- The official C++ path builds from an installed package using only promoted
  headers and package targets.
- The official Python path imports as `dartpy.simulation`, has generated stubs,
  and does not duplicate nanobind class identities.
- Public API boundary checks reject ECS storage, component types, solver
  registries, backend types, and implementation folders.
- In-repo examples, tutorials, tests, and docs use the official path unless they
  are explicitly testing a migration alias.
- DART 6/gz-physics compatibility expectations are documented on the DART 6.16
  lane and do not block the DART 7 main-branch API shape.
- Source-tree moves happen after facade promotion and are isolated enough that
  active solver and loader branches can merge latest main without broad manual
  conflict surgery.

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
- installed-package C++ and wheel import smokes for package/export changes
- reduced-build checks while `DART_BUILD_SIMULATION_EXPERIMENTAL` exists

## Revision Triggers

- A maintainer chooses a different final C++ namespace, Python import path, or
  CMake component shape.
- Parity gates expose a blocker that prevents official promotion in DART 7.
- gz-physics or Gazebo compatibility requires a DART 6.16 support-lane update.
- A physical move PR conflicts heavily with active solver/model-loading work and
  needs to be split or delayed.
