# PLAN-042: DART 7 Public API And Source Layout

- Operating state: `PLAN-042` in [`dashboard.md`](dashboard.md)
- Outcome: DART 7 has an accepted public API topology and source-layout map for
  the official World API. The common user path is deliberately designed instead
  of inherited from DART 6-era submodules or the retired
  `simulation_experimental` staging surface; physical source moves follow the
  public topology, package exports, examples, and verification gates.
- Current evidence: PR #2932 landed the first official DART 7 simulation API
  promotion; the north star requires an easy public API, extensible internals,
  and scalable compute; dartpy now promotes the ECS-backed World through
  `dartpy.simulation.World` and the root `dartpy.World` alias, with generated
  stubs no longer publishing `dartpy.simulation_experimental`;
  `dart::simulation`, `dart/simulation/**`, the `dart-simulation` component, and
  `python/dartpy/simulation/` now own the promoted World surface. The current
  `dart/simulation` folder tree is accepted as a guarded post-promotion layout,
  not as the final whole-repo source taxonomy. Current `dart/*` folders still
  reflect some historical DART 6 module boundaries, backend adapters, and
  compatibility surfaces, so the remaining work is cleanup, quarantine, and
  enforcement rather than choosing staging names.

## Direction

Keep the public topology explicit. Move files only when the move reinforces
that topology and keeps the gates green.

DART 7 should not revive `dartpy.simulation_experimental.World` or
`dart::simulation::experimental::World` as final user-facing names. Those names
were useful staging surfaces. They are historical evidence, not the final API.

The default recommendation to review, then accept or revise, is:

- Python common path: `import dartpy as dart` and construct the first simulation
  through `dart.World(...)`. This is a root convenience, not a second owner:
  `dart.World is dart.simulation.World` must hold.
- Python canonical owner: `dart.simulation` owns the full official World API and
  related simulation object model. The top-level `dart.World` is an alias to
  that owner, not a separate nanobind registration.
- Python teaching path: user-facing docs and first-page examples teach the
  stable top-level package shape:

  ```python
  import dartpy as dart

  world = dart.World()
  ```

  Users who need an explicit module path can use `dart.simulation.World`, which
  is the same class.

- Python advanced path: curated submodules remain discoverable for ownership and
  advanced workflows, but first-time examples should not require users to learn
  `dart.simulation_experimental`, DART 6 legacy modules, backend names, or
  implementation folders. Advanced groups such as `simulation.state`,
  `simulation.compute`, or `simulation.diff` appear only after their C++ public
  wrappers are stable.
- Python root facade: replace accidental broad root flattening with an approved
  allowlist. The initial simulation allowance is `World`; add `WorldOptions` or
  `WorldConfig` only if the first-page example needs it. Do not root-promote
  `simulation_experimental`, backend names, solver registries, parser-specific
  classes, ECS/storage concepts, or research-algorithm staging types.
- C++ common path: keep module ownership explicit unless review proves a
  top-level `dart::World` facade removes real friction without creating ABI,
  include, or ownership confusion. The minimum DART 7 requirement is that no
  public C++ example uses `experimental` names, implementation folders, or
  backend/component storage.
- C++ simulation path: the default review hypothesis is that first-class public
  simulation types live in `dart::simulation` with small snake_case headers such
  as `dart/simulation/world.hpp`. Do not reintroduce
  `dart::simulation::experimental`, add a public transition namespace, or expose
  public `body/`, `frame/`, `multibody/`, `ecs/`, or `comps` namespaces just to
  mirror today's staging folders.
- C++ package path: the official World must be usable from the default DART 7
  package without a staging build flag, the `dart-simulation-experimental`
  component, recursive staging header installs, or public EnTT/Taskflow/CUDA
  dependency discovery. The current package owner is the `dart-simulation`
  component; keep implementation-only dependencies private.
- Source tree: final folders should communicate public concepts and internal
  ownership. Historical folders may remain only when they still describe a
  supported DART 7 concept; otherwise they are quarantined, folded into a better
  owner, or made internal.

The plan should produce a decision record, not a mass rename. A folder move is
aligned only when it follows the accepted topology and has package, docs, and
boundary evidence.

The initial decision and audit packet lives in
[`042-dart7-public-api-and-source-layout/api-source-layout-audit.md`](042-dart7-public-api-and-source-layout/api-source-layout-audit.md).
The post-promotion folder decision lives in
[`042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md`](042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md).
Keep those sidecars focused on concrete rows, gates, and follow-up work items;
keep this owner plan focused on direction, sequencing, and acceptance criteria.

## Review Board

Before implementation, collect and reconcile these review perspectives:

- **API design**: common-path examples, symbol budget, naming clarity, and
  whether top-level names or submodules best serve first-time users.
- **C++ architecture**: namespace/header/folder ownership, export macros, ABI
  effects, generated compatibility headers, and internal/detail boundaries.
- **Python packaging**: import path, `dart.World` behavior, `__all__`, stubs,
  generated docs, wheel imports, feature-on/off build behavior, and legacy
  module removal.
- **Build/package**: final CMake targets/components, installed headers,
  package-config files, dependency leakage, wheel defaults, reduced-build
  behavior, and source-build/install smokes.
- **Research extensibility**: where algorithm families, state spaces, compute
  stages, solver choices, and multi-physics components live without exposing
  backend or storage implementation.
- **Migration/release**: `release-6.*` compatibility lane, examples/tutorials,
  changelog/migration notes, and downstream canaries.

External API surveys may inform the review, but DART-owned docs should record
the resulting principles and DART vocabulary, not the source projects' names.

## Decisions To Make

1. **Python namespace topology.** Choose the DART 7 public contract for
   `dart.World(...)`, top-level symbol promotion, submodule visibility, legacy
   submodule removal, and whether any compatibility aliases remain. The default
   review hypothesis is a small root facade for first-page examples plus
   canonical owner submodules for full APIs and advanced features. The decision
   record must include the approved root allowlist, `__all__` behavior, generated
   `__init__.pyi` behavior, and the exact treatment of current broad flattening
   in `python/dartpy/_layout.py`.
2. **C++ namespace and header topology.** Decide whether the official World type
   is only `dart::simulation::World`, also exposed through a top-level
   convenience facade, or moved to a different owner. Decide umbrella headers,
   generated compatibility headers, install allowlists, and export macros in the
   same record. Do not hand-author PascalCase public headers such as
   `dart/simulation/World.hpp`.
3. **`dart/*` folder map.** Classify each top-level folder as public module,
   internal foundation, backend adapter, compatibility quarantine, or removal
   candidate. The map must explain why each surviving folder helps DART 7's
   easy-start, extensibility, or scalable-compute goals.
4. **Model-loading ownership.** Keep one public loading front door. `dart/io`
   currently owns unified model loading; simulation-level loaders should become
   world adapters, internals, or IO-owned APIs rather than a duplicate public
   path.
5. **Classic simulation and dynamics boundary.** Decide which DART 6 `World`,
   `Skeleton`, `BodyNode`, constraint, and utility concepts are removed,
   quarantined, wrapped, or promoted as DART 7 concepts.

   **Decision: DART 6 legacy removal staging.** The DART 7 end state is to
   completely remove DART 6 legacy from the DART 7 public contract. Until DART
   6.20+ and any later DART 6 maintenance branch have fully ported the
   capabilities they still need, `main` keeps DART 6 `dart/dynamics`, classic
   constraint, and classic simulation concepts only as a compatibility
   quarantine lane. The DART 6.20+ port lane, plus later `release-6.*` branches
   if needed, owns parity work for DART 6 users; `release-6.*` branches remain
   the support lane and are the place to land DART 6 feature parity or
   downstream compatibility fixes. Current open DART 6.20+ PRs observed at
   WP-091 closeout include #3071, #3086, #3088, and #3092, but the list is not
   exhaustive.

   On `main`, `World`, `Skeleton`, `BodyNode`, classic joint/constraint
   concepts, and legacy utilities do not get new first-use public API under
   their DART 6 names. Surviving concepts must be reintroduced through DART 7
   owners and vocabulary, such as `dart/simulation`, `dart/simulation/multibody`,
   or the curated DART 7 loop-closure/constraint families, with focused owner
   docs and gates. The orphaned cylindrical joint constraint is not revived as a
   legacy symbol; if DART 6.20+ evidence proves the capability is still needed,
   it should be ported as a DART 7 loop-closure or multibody constraint family.
   Otherwise it retires with the DART 6 legacy surface.

   The quarantine lane is frozen on `main`: new public C++ symbols, dartpy
   bindings, or generated stubs in `dart/dynamics`, legacy `dart/constraint`,
   `python/dartpy/dynamics`, `python/dartpy/constraint`, or matching
   `python/stubs/dartpy/*.pyi` files must be rejected unless the exact new
   declaration, binding, or stub carries the local tag
   `dart7-legacy-freeze: bugfix-port` and is justified as a release-6.\* bugfix
   port. That tag is not an approval to promote DART 6 API on `main`; it is a
   narrow marker for unavoidable parity fixes while the port lane is still
   active.

6. **Multi-physics and compute grouping.** Decide where rigid bodies,
   multibodies, deformables, sensors, rendering sync, state spaces, compute
   stages, execution profiles, and backend adapters live so adding a solver or
   backend does not fork the user API.
7. **Package and documentation topology.** Decide final CMake targets,
   installed headers, dartpy stubs, API docs, examples, and tutorial paths
   before source moves make those names expensive to change.
8. **Differentiable helper home.** Keep `dart.diff` and `dart.simulation.diff`
   aligned while deciding whether differentiable helpers remain
   simulation-specific or move to a separate stable owner. Any reintroduced
   compatibility alias must be temporary and covered by identity, import, stub,
   and docs tests.
9. **Public-layout checker.** Decide whether to add a dedicated
   `check-dartpy-public-layout` gate or fold the same runtime/stub/docs checks
   into `check-api-boundaries`.
10. **Package compatibility policy.** Decide how DART 7 package config/version
    behavior, wheel defaults, migration notes, `release-6.*` support, and gz-physics
    lane policy line up so clean-break packages are not accidentally consumed as
    old DART 6-compatible packages.

## Source Layout Audit Map

The first implementation task derived from this plan is a repo-wide audit map.
The initial map lives in
[`042-dart7-public-api-and-source-layout/api-source-layout-audit.md`](042-dart7-public-api-and-source-layout/api-source-layout-audit.md).
The current post-PR-2932 folder decision and open follow-up rows live in
[`042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md`](042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md).
Each row uses these fields:

| Field                 | Meaning                                                                             |
| --------------------- | ----------------------------------------------------------------------------------- |
| Current owner         | Existing C++ folder, Python module, stub page, target, or docs surface.             |
| Current role          | Public API, compatibility API, retired staging API, backend adapter, or internal.   |
| DART 7 role           | Common public path, advanced public module, internal owner, quarantine, or removal. |
| Proposed public name  | Final import, namespace, header, target, or `n/a`.                                  |
| Proposed source owner | Final folder/module or existing owner that should absorb the concept.               |
| Migration action      | Keep, rename, wrap, move, split, merge, quarantine, or delete.                      |
| Gate                  | Tests, stubs/docs, package smoke, API-boundary check, or conflict preflight.        |
| Open decision         | Maintainer choice or evidence needed before implementation.                         |

Initial folders/surfaces to classify:

| Surface                                                                 | Current concern                                                                                        | Review question                                                                                          |
| ----------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------- |
| `dart/common`                                                           | Mixes public utilities and internal foundation patterns.                                               | Which pieces are true DART 7 user API versus internal support?                                           |
| `dart/math`, `dart/simd`                                                | Strong foundations, but not every kernel belongs in first-use docs.                                    | Which symbols are common path, advanced math, or internal acceleration?                                  |
| `dart/dynamics`                                                         | DART 6 object model and naming dominate this folder.                                                   | Which concepts survive as DART 7 public multibody API, and which move behind compatibility or internals? |
| `dart/simulation`                                                       | Promoted World owns the official name, while internal subtrees must stay hidden.                       | Which files are public, internal, IO-owned, compute-owned, or deleted?                                   |
| `dart/simulation/experimental`                                          | Retired staging path.                                                                                  | Negative checks should keep it from returning as a public include path.                                  |
| `dart/io`, `dart/utils`                                                 | `io` is the intended model-loading front door, while parser code still spans historical utility paths. | How do loading APIs consolidate without duplicate front doors?                                           |
| `dart/collision`, `dart/constraint`, `dart/lcpsolver`, `dart/optimizer` | User concepts and solver machinery are mixed across old module boundaries.                             | Which extension contracts remain public, and which solver/backend details become internal?               |
| `dart/gui`, `dart/sensor`                                               | Runtime integrations should stay backend-hidden.                                                       | Which viewer/sensor concepts are public versus application or backend internals?                         |
| `python/dartpy`                                                         | Top-level API coexists with legacy submodules and the promoted `simulation` module.                    | What is the DART 7 top-level symbol budget and submodule compatibility story?                            |
| `python/stubs`, generated API docs                                      | Generated surfaces still encode current module decisions.                                              | Which generated pages prove the accepted topology?                                                       |

## C++ Renovation Constraints

The C++ simulation source tree should be renovated in stages:

- **Move or expose early**: accepted header/module map,
  `check-simulation-public-header-allowlist`, promoted-header boundary checker,
  package export contract, and thin storage-neutral facade headers or value
  objects.
- **Do not promote yet**: handles or classes that still expose entity IDs,
  registry ownership, ECS base classes, `comps/`, or `ecs/` includes.
- **Move later**: compute subtrees, loader bridges, solver detail trees, and
  deletion of obsolete build options/components when the owner and gate for each
  subtree is clear.

Public C++ simulation headers must pass these checks before promotion:

- installed by allowlist rather than recursive implementation-tree install;
- no `<entt/entt.hpp>`, `comps/`, `ecs/`, direct registry access, public entity
  IDs, public ECS helper inheritance, solver-kernel types, backend adapters, or
  implementation-folder includes;
- exported with the final component macro rather than a staging macro;
- usable from an installed package without a staging build option or a
  `simulation-experimental` target;
- usable from wheels and default source builds without staging build flags;
- guarded by `check-dart7-world-promotion-blockers` so any remaining
  main-branch DART 6 World, experimental namespace, staged target/component, or
  parity-reference dependency is explicit and can be driven to zero before the
  final public shape is accepted;
- validated on a case-insensitive filesystem when generated compatibility
  headers or `world.hpp`-adjacent paths change.

## Python Import Compatibility Matrix

The accepted Python decision must define each row before implementation:

| Surface                                            | DART 7 outcome to decide                                                  | Required evidence                                                                   |
| -------------------------------------------------- | ------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| `import dartpy as dart`                            | Canonical package import taught in user docs.                             | Docs/examples use the final first-use path.                                         |
| `dart.World`                                       | Root convenience for the official simulation World.                       | `dart.World is dart.simulation.World`; root `__all__` includes it only if approved. |
| `dart.simulation.World`                            | Canonical owner path for the full official World API.                     | Same class identity as the root facade, stubs, and docs.                            |
| `dartpy.simulation.World`                          | Same canonical owner, using the package name directly.                    | Import smoke and wheel smoke.                                                       |
| `dart.simulation_experimental`                     | Absent unless a maintainer approves a Python-level migration alias.       | No second C++ binding; documented identity behavior if alias exists.                |
| `from dartpy import simulation_experimental as sx` | Unsupported unless a migration alias is explicitly reintroduced.          | Import smoke, warning expectation, and removal condition if retained.               |
| `sx.World`                                         | Compatibility alias only if `sx` returns.                                 | `sx.World is dart.simulation.World`; no duplicate `nb::class_`.                     |
| `sx.diff`                                          | Compatibility alias only if `sx` returns.                                 | Stub/doc/runtime parity, lazy torch behavior, and final-home decision if retained.  |
| `dart.simulation.diff`                             | Preferred final owner if differentiable helpers stay simulation-specific. | Runtime import, generated stubs, generated docs, and torch-free import smoke.       |
| `__all__`                                          | Matches the accepted root symbol allowlist.                               | Focused runtime test and generated `__init__.pyi` check.                            |
| `sys.modules` aliases                              | Only final paths plus documented compatibility aliases.                   | Focused import-layout test.                                                         |
| Generated stubs/docs                               | Encode the same layout as runtime.                                        | `generate-stubs`, `api-docs-py`, and docs policy gates.                             |
| Default-build public World                         | Official Python World exists without staging build flags.                 | Source/wheel smoke with the default DART 7 configuration.                           |

## Sequencing

1. **Decision record now.** Treat the Python namespace decision as implemented
   unless maintainers revise it: `dart.World is dart.simulation.World`,
   generated stubs expose that path, `dart.simulation_experimental` is absent,
   and `dart.diff is dart.simulation.diff`. Keep the C++ and package decision
   explicit: `dart::simulation::World`, no initial C++ `dart::World` facade,
   final header/source owner, `dart-simulation` target/component shape, and
   DART 6 quarantine/removal policy.
2. **Small facade PRs next.** Add or adjust public facades, package smokes,
   stubs, docs, `check-simulation-public-header-allowlist`,
   `check-dart7-promotion-package-contract`,
   `check-dart7-promotion-installed-package`, `check-dartpy-import-layout` or
   equivalent public-layout coverage, `check-dart7-world-promotion-blockers`,
   and boundary checks using the accepted names.
3. **Official World promotion.** PLAN-041 consumes the accepted PLAN-042
   topology to keep the promoted World path, package target, and Python identity
   coherent.
4. **Mechanical source renovation.** Move remaining files in small batches by
   owner: public headers, internals, Python bindings, docs, generated
   compatibility headers, then cleanup aliases/options.
5. **Retire the plan.** Once durable design/onboarding docs, code, examples,
   and package checks own the result, mark PLAN-042 complete and remove or
   consolidate this numbered plan.

## Acceptance Criteria

- A DART 7 namespace decision record chooses the Python common path, Python
  advanced submodule policy, C++ World namespace/header policy, and top-level
  package/import behavior, including whether any root alias beyond `World` is
  permitted in the first promotion.
- The accepted root facade is an explicit allowlist. Runtime `__all__`,
  generated `__init__.pyi`, and generated API docs match that allowlist and do
  not accidentally preserve whole-module flattening from `python/dartpy/_layout.py`.
- The public first-use example uses final names and does not mention
  `experimental`, DART 6 legacy modules, backend names, component storage, or
  implementation folders.
- The accepted Python policy proves `dart.World is dart.simulation.World`, does
  not bind a second C++ World class, keeps
  `dart.simulation_experimental` absent unless a maintainer explicitly
  reintroduces a Python-level alias with documented identity behavior,
  deprecation behavior, and removal condition.
- The Python import compatibility matrix covers runtime imports, `__all__`,
  `sys.modules`, stubs, generated docs, wheel import smokes, and the final home
  for differentiable helpers exposed through `dart.diff` and
  `dart.simulation.diff`, plus any explicitly reintroduced compatibility alias.
- Advanced Python submodules are importable, documented, stubbed, and omitted
  from broad root flattening unless explicitly approved.
- The repo-wide audit map classifies every top-level `dart/*` folder,
  `python/dartpy` submodule, generated stub/API-doc surface, and package target
  touched by DART 7 API promotion, starting with the PLAN-042 audit sidecar.
- PLAN-041 points to this plan for global API/source-layout decisions and owns
  only the official simulation API promotion sequence.
- Implementation follow-ups have explicit gates for API-boundary checks, stubs,
  docs, C++/Python tests, public-layout checks or equivalent API-boundary
  coverage, package/export smokes, case-insensitive header checks, and conflict
  preflights for broad moves.
- The package decision proves the official World from a reduced source build, an
  installed C++ package, and a release wheel without the old experimental option
  or component, and scans installed config/component files for private dependency
  leakage.
- DART 7 package metadata, migration notes, `release-6.*` support window, and
  gz-physics lane policy agree before release publication.

## Verification Gates

For this planning surface:

- `pixi run lint-md`
- `pixi run check-lint-md`
- `pixi run check-docs-policy`
- `pixi run check-lint-spell`
- `pixi run lint`

For implementation PRs derived from this plan, select by touched scope:

- `pixi run check-api-boundaries`
- `pixi run check-dart7-promotion-surface` and
  `pixi run check-simulation-public-headers` for the promoted C++ header
  allowlist
- `pixi run check-dart7-promotion-package-contract` for the package
  target/component contract
- `pixi run check-dart7-promotion-installed-package` for the local-install
  package smoke
- `pixi run check-dartpy-import-layout` for runtime/static Python import
  layout, stubs, aliases, and `sys.modules`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-py`
- `pixi run generate-stubs`
- `pixi run api-docs-py`
- installed-package C++ and wheel import smokes
- default source/wheel smokes proving the final World path without staging
  build flags
- negative smokes for removed headers, modules, targets, and aliases
- `pixi run check-dart7-final-world-promotion`, with parity evidence sourced
  from local `release-6.*` branch refs rather than main-branch DART 6 World tests
- case-insensitive filesystem validation for renamed public headers

## Revision Triggers

- Maintainers choose a different Python import/package policy.
- Maintainers choose a different C++ namespace/header policy.
- PLAN-041 promotion exposes a conflict with the accepted topology.
- Package smokes show that a proposed source move leaks private dependencies or
  breaks installed consumers.
- A top-level folder audit finds a DART 6-era module boundary that blocks
  easy-start examples, algorithm extension, or scalable compute.
