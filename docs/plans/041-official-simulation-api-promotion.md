# PLAN-041: Official Simulation API Promotion

- Operating state: `PLAN-041` in [`dashboard.md`](dashboard.md)
- Outcome: the ECS-backed simulation world becomes the official DART 7 C++ and
  dartpy simulation API, with the experimental namespace/module removed from the
  promoted public contract and the source tree reorganized only after the stable
  facade is in place. DART 6 API compatibility remains on the DART 6.16 support
  lane; main moves as a DART 7 clean break.
- Current evidence: the experimental world already owns the DART 7 simulation
  direction, but its current public shape still exposes experimental paths,
  implementation-oriented headers, recursive header installs, build-option
  seams, public package-interface leaks, and a Python module split that cannot
  be treated as the final API without a deliberate promotion pass.

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

External architecture review informs the plan, but DART should not copy another
engine's vocabulary or package shape. The reusable pattern is a small common
front door, explicit lifecycle/state concepts, solver and backend choices behind
domain names, and optional advanced surfaces grouped only after their owner APIs
are stable.

Repo-wide public API topology and source-folder renovation are owned by
[`042-dart7-public-api-and-source-layout.md`](042-dart7-public-api-and-source-layout.md).
This plan consumes that decision for the simulation World cutover; it should not
freeze global dartpy root-promotion policy or whole-repo `dart/*` folder
renames on its own.

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
  namespace/target; it must not keep exporting `dart::simulation::World`. The
  promotion needs an explicit header map before any source move because the
  current staging tree recursively installs public-looking `detail/`, `comps/`,
  and `ecs/` headers.
- **Python bindings**: the likely promoted import path is `dartpy.simulation`,
  with `dartpy.simulation_experimental` reduced to a short compatibility alias
  only if needed. PLAN-042 owns the broader root facade policy; its current
  working hypothesis is that first-use examples can use `dart.World(...)` after
  `import dartpy as dart`, while `dart.simulation.World` remains the canonical
  owner and object identity. The cutover is a single import-layout transaction
  across `_dartpy` submodule registration, `_layout.py`, `sys.modules`,
  `__all__`, stubs, generated docs, and the pure-Python `diff` attachment. Avoid
  duplicate nanobind class registration: compatibility aliases must forward to
  the promoted Python object rather than binding a second C++ `World` type into
  `dartpy.simulation`. Do not repoint top-level `dartpy.World` without an
  explicit maintainer decision because that name currently implies the legacy
  simulation module. The promotion must decide whether `dartpy.World` is removed,
  deprecated, or identical to `dartpy.simulation.World`, and gate that decision
  with tests, stubs, generated docs, and wheel import smokes.
- **Build, package, and downstream**: decide whether the promoted simulation API
  is a core component or a separately discoverable CMake component before
  reshaping install/export rules. The official baseline API must be present in
  default builds before promotion is claimed; it cannot remain hidden behind the
  old `DART_BUILD_SIMULATION_EXPERIMENTAL` option or require a
  `simulation-experimental` target to link. The final installed package must not
  force implementation-only dependencies through public CMake targets or
  component files. Keep reduced-build and package smoke tests in the gate while
  the old option still exists.
- **Scalar precision**: DART 7 promotion keeps the public `World` facade
  double-backed. Public scalar-type support is deferred until the promoted
  rigid-body and multibody stack is in good shape for humanoid locomotion and
  manipulation. The promotion work should keep scalar support from becoming a
  one-way door by avoiding unnecessary storage, handle, package, and API-boundary
  commitments, but it should not add `sx.World(dtype=...)` or
  `sx.World[...]` during promotion unless a dedicated scalar-instantiation plan
  proves C++ ownership, bindings, stubs, serialization, collision,
  differentiability, and package gates for every advertised scalar. If that
  future plan makes precision public, the primary Python construction spelling
  is `sx.World(dtype=sx.float64)` (or `WorldOptions(dtype=...)`) with concrete
  scalar-specific implementation classes underneath as needed;
  `sx.World[sx.float64]` is not the common runtime API.

## Workstreams

1. **Promotion contract and readiness audit** - freeze the initial supported
   public subset, record the headers/modules to promote, list public-looking
   internals to hide, add the header promotion map described below, and map
   parity evidence that blocks the promotion claim. Consume PLAN-042's accepted
   namespace/source-layout decision before freezing final public names. This step
   also decides whether PLAN-042's `check-simulation-public-header-allowlist`
   and `check-dartpy-import-layout` are new scripts or extensions of existing
   API-boundary checks, and records the scalar-precision policy from the
   simulation experimental API design docs; it does not move source files.
2. **Build and package shape** - choose the final target/component/export-macro
   shape before the first promoted API PR. Make the official baseline API
   non-optional in default builds, update install/export rules, and add package
   smokes that prove promoted headers and Python modules are available without
   experimental build flags. Inspect installed targets and component files so
   promoted exports do not require implementation-only dependency discovery.
3. **Promoted API boundary enforcement** - update boundary inventories and
   checks so promoted C++ headers fail if they expose EnTT, ECS storage,
   component namespaces, solver registries, backend types, implementation
   folders, direct registry access, public entity IDs, or public inheritance from
   ECS helper templates. Add a promoted-simulation-header manifest/checker
   (`check-simulation-public-header-allowlist` or equivalent
   `check-api-boundaries` coverage), add a promotion-aware inventory
   classification, and add negative installed-package smokes for forbidden
   headers and obsolete experimental targets.
4. **World name-collision transaction design** - choose how to clear the classic
   `dart::simulation::World` collision before implementation. Either use a
   temporary non-colliding facade while consumers migrate, or replace the classic
   `World` definition and introduce the ECS-backed facade atomically. A
   quarantine plan must not continue exporting `dart::simulation::World`.
5. **C++ official facade** - introduce the promoted C++ API under the chosen
   namespace and public include path. Use wrappers or opaque implementation
   ownership where needed so public headers do not expose ECS internals. This is
   a semantic facade step; broad physical `git mv` work waits until later.
   Confirm top-level module ownership before moving loaders, collision helpers,
   sensors, or GUI synchronization code so DART 7 does not create duplicate
   public front doors.
6. **Python official facade** - bind the promoted API under `dartpy.simulation`,
   decide the top-level `dartpy.World` outcome, generate stubs and docs for the
   final path, and make `dartpy.simulation_experimental` a Python-level
   compatibility alias only if the release path needs it. Cover `sx.diff`,
   `_layout.py`, `_naming.py`, `sys.modules`, `__all__`, GUI stubs, generated API
   pages, `check-dartpy-import-layout` or equivalent boundary coverage, and any
   alias window in the migration matrix.
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

## DART 7 Simulation Source And Module Layout Contract

The DART 7 source layout should be redesigned around the public API users should
see, not around the current staging tree. The contract for implementation PRs is:

- Promoted user headers live under the final `dart/simulation/` public layout
  and include only public DART headers or standard/library dependencies that are
  part of the final contract.
- ECS storage, component definitions, solver kernels, backend adapters,
  registry access, and loader internals live under `detail/`, `internal/`, or an
  existing non-public owner. Recursive installation of the staging tree is
  forbidden once promotion starts.
- Public handles are opaque. Promoted headers must not include EnTT, expose
  entity IDs, include `comps/` or `ecs/`, inherit publicly from ECS helper
  templates, or require users to reason about registry ownership.
- Existing top-level owners stay authoritative: `dart/io` remains the unified
  model-loading front door, `dart/collision` owns collision-query concepts,
  `dart/sensor` owns sensor concepts, and `dart/gui` owns GUI/viewer concepts.
  `dart/simulation` may own world-level adapters that compose those modules, but
  it should not duplicate their public entry points.
- The final package exposes a named CMake target/component and export macro
  choice. Promotion is not claimable while a consumer must enable
  `DART_BUILD_SIMULATION_EXPERIMENTAL`, link `dart-simulation-experimental`, or
  discover implementation-only dependency packages to use the official API.
- The Python layout promotes one object identity. `dartpy.simulation.World`,
  the maintainer-approved `dartpy.World` behavior, any
  `dartpy.simulation_experimental` alias, generated stubs, generated docs,
  `__all__`, and pure-Python helpers move together. If an alias remains, it
  forwards at Python level to the promoted class object and does not bind a
  second C++ `World` class into `dartpy.simulation`.

The promotion readiness audit must include a header/module map with these
fields:

| Field                 | Meaning                                                                     |
| --------------------- | --------------------------------------------------------------------------- |
| Current path          | Existing C++ header/source or Python binding/stub/helper path.              |
| Final public path     | Final user-facing include or import path, or `n/a` if internal.             |
| Final internal owner  | `detail/`, `internal/`, existing top-level module, or removal target.       |
| Public class          | Promoted public, compatibility alias, internal-only, or removed.            |
| CMake/export outcome  | Final target/component/export macro or private implementation target.       |
| Docs/install outcome  | Included in user docs/install, internal docs only, or not installed.        |
| Alias/removal outcome | Kept, Python-level alias, C++ forwarding header, or deleted.                |
| Gate                  | Boundary, package, stub/doc, test, conflict, or platform evidence required. |

Do not land a physical move PR until this map is accepted for the moved surface.
The map is allowed to be incremental by workstream, but each moved file must have
an entry before it moves.

## World Name-Collision Transaction Path

The durable rationale lives in
[`../design/dart7_world_namespace_transaction.md`](../design/dart7_world_namespace_transaction.md).
That design recommends option (b): atomically replace the classic
`dart::simulation::World` public definition with the ECS-backed facade, after
sequencing the risky prerequisites independently. The plan owns the active
sequence, per-step gates, and open decisions for that transaction.

Open maintainer decisions before implementation:

- Confirm option (b) atomic replacement over option (a) temporary facade.
- Confirm the facade lifecycle shape before the name swap: re-add
  `WorldPtr`/`create`/`clone` shared ownership, move consumers to value
  semantics, or choose another explicit owner model.
- Confirm `dartpy.World` as identical to `dartpy.simulation.World`
  (recommended) versus a deprecate-then-remove transition.
- Confirm the classic-world quarantine target name and that it stops exporting
  `dart::simulation::World`, or remove the classic implementation outright.
- Confirm no public scalar-precision selector is added during DART 7 promotion.
  A later scalar-instantiation plan may reopen precision after the DART 7
  rigid-body and multibody baseline is ready for humanoid locomotion and
  manipulation, but must keep the promoted `World` identity stable and prove its
  own scalar-specific gates first.

Current intended sequence:

1. **Source-layout prerequisite (workstreams 1, 2, 3, and 5).** Accept the
   header/module map for the surfaces touched by the name swap; prove the public
   facade compiles from installed headers; exclude internal headers from install
   and generated user docs; verify the final target/component/export interface;
   preflight active solver and loader branch conflicts; and validate the
   `world.hpp`/`World.hpp` behavior on a case-insensitive filesystem before the
   atomic replacement PR.
2. **Build-shape prerequisite (workstream 2).** Make the ECS-backed baseline
   non-optional in default builds and keep EnTT/ECS out of the public boundary.
   No public name changes yet. Gate with `pixi run build`,
   `pixi run test-simulation-experimental`, a reduced-build check while
   `DART_BUILD_SIMULATION_EXPERIMENTAL` still exists, and an installed-package
   C++ smoke.
3. **Boundary enforcement prerequisite (workstream 3).** Update
   `scripts/check_api_boundaries.py` and
   `scripts/generate_api_boundary_inventory.py` so promoted headers fail if they
   expose EnTT, `entt::registry`, ECS storage, component namespaces, solver
   registries, backend types, public entity IDs, public ECS helper inheritance,
   or direct registry access. The current `getRegistry()` escape hatch and any
   equivalent storage access must be hidden from the promoted facade. Gate with
   `pixi run check-api-boundaries` and
   `pixi run check-api-boundary-inventory`.
4. **C++ facade prerequisite (workstream 5).** Introduce the public ECS-backed
   facade behind a non-colliding internal/interim symbol, with opaque ownership
   or wrappers so no public header includes `<entt/entt.hpp>`. Reconcile the
   lifecycle model before the public name swap. Gate with `pixi run build`,
   `pixi run test-unit`, `pixi run test-simulation-experimental`, and
   `pixi run check-api-boundaries`.
5. **Atomic C++/Python name-swap PR (workstreams 4, 5, 6, 7, and 8).** Replace
   or repoint `dart/simulation/world.hpp` and `dart/simulation/fwd.hpp` so
   `dart::simulation::World` resolves to the ECS facade and `fwd.hpp` reflects
   the maintainer-approved lifecycle model, keeping `WorldPtr` only if shared
   ownership is selected; quarantine or remove the classic implementation so it
   no longer exports `dart::simulation::World`; rebind `simulation.World` from
   the classic type to the ECS facade in `python/dartpy/simulation/world.cpp`;
   remove the old
   `nb::class_<dart::simulation::World>(m, "World")` in the same change so the
   `simulation` module never holds two C++ types under the name `"World"`; and
   regenerate stubs/docs for the final Python shape, including
   `simulation.pyi`, `simulation_experimental.pyi` if an alias remains,
   `__init__.pyi`, GUI stubs that mention world types, and generated API pages.
   Retarget all 158 matched in-tree consumer files: 35 under `dart/`, 1 under
   `dartsim/`, 47 under `examples/`, 4 under `python/`, and 71 under `tests/`.
   Keep the public header name `dart/simulation/world.hpp`; do not hand-author
   `dart/simulation/World.hpp`; and preflight active solver and loader PR heads
   with `git merge-tree`. Gate with `pixi run build`, `pixi run test-unit`,
   `pixi run test-simulation-experimental`, `pixi run test-py`,
   `pixi run generate-stubs`, `pixi run api-docs-py`,
   `pixi run check-api-boundaries`, `pixi run check-api-boundary-inventory`, a
   macOS case-insensitive-filesystem build, an installed-package C++ smoke, and
   a wheel import smoke.
6. **Compatibility alias cleanup (workstreams 6 and 10).** Decide and then
   retire any `dartpy.simulation_experimental` alias window, obsolete
   `DART_BUILD_SIMULATION_EXPERIMENTAL` option, stale package component, or
   migration shim left behind by the atomic name-swap PR. Do not defer the
   `dartpy.simulation.World` rebind itself past the atomic C++/Python name-swap
   state. Gate with `pixi run test-py`, `pixi run generate-stubs`,
   `pixi run api-docs-py`, package/export smokes, and a wheel import smoke.
7. **Compatibility documentation (acceptance + clean-break strategy).** Record
   DART 6 / gz-physics support expectations on the DART 6.16 lane, including
   the Gazebo branch/version matrix, support window, and sunset trigger in the
   compatibility owner docs referenced by
   [`../design/dart7_clean_break_strategy.md`](../design/dart7_clean_break_strategy.md)
   before main removes the classic surface.

## High-Risk Surfaces

Review these before implementation PRs:

- `dart/simulation/world.hpp` and the classic `dart::simulation::World`
  namespace collision.
- `dart/simulation/experimental/world.hpp`, especially exposed registry or ECS
  access.
- `dart/simulation/experimental/ecs/` and `comps/` headers that look public only
  because they are installed today.
- Handle headers whose constructors, base classes, accessors, includes, or
  layout reveal ECS identity or component storage.
- `dart/simulation/experimental/*_loader.*`, frame/body/joint handles, and stage
  APIs that may currently reveal implementation ownership.
- `dart/simulation/experimental/CMakeLists.txt`, root CMake options, package
  exports, installed target/component files, public dependency discovery, and
  workflow path filters.
- `dart/simulation/CMakeLists.txt` generated component headers and PascalCase
  compatibility headers, especially `World.hpp`/`world.hpp` case-insensitive
  filesystem collisions.
- `python/dartpy/dartpy.cpp`, `python/dartpy/simulation/`,
  `python/dartpy/simulation_experimental/`, `_layout.py`, `_naming.py`, stub
  generation, and Python API boundary checks.
- `scripts/check_api_boundaries.py`, `scripts/generate_api_boundary_inventory.py`,
  `scripts/generate_stubs.py`, and generated API docs that currently know about
  experimental module names.

## Acceptance Criteria

- The official C++ path builds from default source builds and from an installed
  package using only promoted headers, final package targets, and final export
  macros.
- The header/module map covers every moved or promoted file, and no promoted
  installed header remains under `experimental/`, `comps/`, `ecs/`, `detail/`,
  or another implementation-owned path.
- Installed CMake targets/components for the promoted API do not expose
  implementation-only dependencies, obsolete experimental components, or
  private include directories through their public interface.
- The official Python path imports as `dartpy.simulation`, has generated stubs
  and generated docs, and does not duplicate nanobind class identities.
- The top-level `dartpy.World` decision is explicit: removed, deprecated, or
  identical to `dartpy.simulation.World`, with tests, stubs, docs, and wheel
  import smokes covering the chosen behavior.
- The promoted DART 7 `World` API is documented as double-backed. No
  `sx.World(dtype=...)`, `sx.World[...]`, scalar-specific `World` aliases, or
  public C++ scalar-template facade is shipped unless a separate
  scalar-instantiation plan supplies concrete ownership, dtype, identity,
  serialization, collision, differentiability, package, and migration gates
  after the DART 7 rigid-body and multibody locomotion/manipulation baseline is
  strong enough to make scalar precision the next bounded design priority.
- If migration aliases exist, the compatibility matrix covers
  `import dartpy.simulation`, `import dartpy.simulation_experimental`,
  `from dartpy import simulation_experimental as sx`, `sx.diff`, and
  `isinstance`/identity behavior across aliases.
- Public API boundary checks reject ECS storage, EnTT, component types, solver
  registries, backend types, direct registry access, public entity IDs,
  implementation folders, public ECS helper inheritance, and promoted headers
  that include internal-only paths.
- Installed-package negative smokes reject forbidden experimental headers and
  obsolete `simulation-experimental` package targets once promotion is claimed.
- Python module-layout tests cover `dartpy.simulation.World`,
  `dartpy.World`, any `dartpy.simulation_experimental` alias,
  `sys.modules`, `__all__`, generated stubs, GUI stub references, `sx.diff` or
  its final home, reduced-build behavior while the old option exists, and wheel
  import behavior.
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
