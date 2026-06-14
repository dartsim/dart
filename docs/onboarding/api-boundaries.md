# Public and Internal API Boundaries

## Start here next time

- Adding a user-facing C++ type? Put it in a public module header, document the
  behavior, export it with the component `*_API` macro if it crosses a shared
  library boundary, and add tests.
- Adding implementation machinery? Put it under `detail/` or `internal/`, keep
  it out of `dartpy`, and prefer `*_LOCAL` for non-header-only symbols.
- Binding Python? Expose only the user-facing API unless a legacy exception is
  explicitly allowlisted by `scripts/check_api_boundaries.py` with a
  replacement and removal condition.
- Changing DART 6 compatibility API? Run the Gazebo workflow from
  [build-system.md](build-system.md#gazebo-integration-feature) on the affected
  support branch.
- Changing a `detail` or `internal` type that appears in an installed public
  signature, base class, alias, or layout? Treat it as exposed implementation
  compatibility debt until the public replacement lands.

## Why This Boundary Exists

DART 7 intentionally refactors large parts of the codebase as a clean break
from the DART 6 API. Released downstream compatibility where there is evidence,
especially gz-physics, is maintained on `release-6.*` unless a maintainer
explicitly scopes a bounded migration adapter. The project needs a small,
stable user-facing API and a larger internal API that can change quickly for
dependency removal, collision backend work, performance, and new simulation
features.

The boundary is not defined by one file name or one macro. A DART API is public
only when the header layout, documentation, tests, symbol visibility, and
language bindings all treat it as supported.

The enforced policy source of truth is `scripts/check_api_boundaries.py` plus
`scripts/check_api_boundaries_allowlist.txt`; run `pixi run check-api-boundaries`
to verify the boundary policy after public header or dartpy binding changes. An
on-demand, human-readable signal report can be produced with
`pixi run report-api-boundary-inventory` (written to an untracked build path and
not committed).

## API Categories

### Supported Public API

Supported public API is what users should build new code against. It must be
hard to misuse, documented, covered by tests, and stable within the current
major release line.

Signals:

- Installed header outside `detail/` or `internal/`.
- Public namespace such as `dart::dynamics`, `dart::simulation`, `dart::io`, or
  another documented module namespace.
- Component export macro such as `DART_API`, `DART_GUI_API`, or `DART_IO_API`
  when the symbol must cross a shared library boundary.
- Doxygen or user-guide documentation that describes behavior, ownership,
  lifetime, and error handling.
- Python binding when the concept is appropriate for end users.

### Compatibility API

Compatibility API is retained for existing users or downstreams on a documented
support lane even when it is not the preferred DART 7 interface. Keep it
source-compatible until the documented removal point, add `DART_DEPRECATED` or
`[[deprecated]]` where possible, and mention the replacement. Gz-physics
compatibility shims belong on `release-6.*` by default; a DART 7 shim must be
bounded, inventory-driven, and sunset dated.

Each compatibility entry needs downstream evidence, replacement API, first
deprecated release, earliest removal release, blocking migration condition, and
release-note or changelog requirement. For gz-physics on the `release-6.*` support
lane, removal requires evidence from the pinned gz-physics integration or an
accepted upstream migration; a version target alone is not enough.

### Exposed Implementation Compatibility Debt

Some installed public headers currently expose implementation types through
public bases, aliases, template arguments, method signatures, or object layout.
Those types remain bad public API, but they are not freely changeable just
because the spelling contains `detail` or `internal`.

Before changing exposed implementation debt:

- Inventory the owning public header, exposed type, and affected C++/Python
  symbols.
- Provide a supported replacement API or wrapper and migration notes.
- Run the downstream gate for touched support-lane surfaces, especially
  gz-physics on `release-6.*`.
- Review ABI risk when exported class layout, virtual bases, inline methods, or
  exported data are involved.

### Experimental API

Experimental API can change faster, but it still needs docs, tests, and clear
ownership. Do not expose implementation-only component storage or backend
plumbing just because the surrounding module is experimental.

For `dart::simulation::compute`, keep the stable-looking surface
small and backend-neutral. The supported compute concepts are graph nodes,
explicit graph dependencies, executor injection, profiles, domain/acceleration
metadata, DOT visualization, and stage/pipeline composition. Do not expose
Taskflow types, GPU devices, streams, memory pools, SIMD storage requirements,
solver registries, or rendering backends until a later plan and benchmark gate
justify them. Future resource read/write declarations should start as
diagnostic metadata and validation inputs before becoming scheduler contracts.

DART 7 deformable simulation follows the same boundary: public handles
and options describe model topology, initial state, fixed nodes, and physical
parameters. Public compute stages may expose broad domain-level hooks such as
deformable dynamics, but ECS components, algorithm-specific stage names,
feasibility barriers, and backend/project names stay internal. Static collision
geometry must be modeled explicitly through public world objects; do not add
hidden default contact surfaces or expose contact-barrier tuning on body options
without a design update.

#### Simulation package shape (`dart::simulation`)

The DART 7 simulation module is built as part of default DART 7
configurations when its required sibling targets are present, and ships only a
small, explicit public subset:

- Only the promoted public headers are installed, via an explicit
  `install(FILES ...)` allowlist in
  `dart/simulation/CMakeLists.txt`
  (`dart_simulation_public_headers`). The implementation-internal directories
  (`comps/`, `compute/`, `common/`, `detail/`, `diff/`, `ecs/`, `io/`,
  `space/`) are not installed. New internal directories are excluded by default;
  promoting a header means adding it to the allowlist on purpose.
- `EnTT` and `Taskflow` are PRIVATE implementation dependencies. They are not in
  the link interface and are not registered as component dependency packages, so
  a downstream `find_package(dart)` does not need or re-run their finders. The
  promoted headers must not include or name them.
- Two guards keep this true:
  `pixi run check-dart7-promotion-surface` (strict source-level audit, fails if
  any promoted header reintroduces an ECS/EnTT leak, raw entity-ID token, or
  public reference to the internal entity-conversion seam) and
  `pixi run check-simulation-public-header-smoke` (a build-tree TU that
  includes every promoted header but is compiled without entt/taskflow on its
  include path, so a leaked `<entt/...>` include fails with "file not found").
  `pixi run check-simulation-public-headers` runs both. Package-shape drift is
  guarded separately by `pixi run check-dart7-promotion-package-contract`, which
  fails if the DART 7 simulation package stops defaulting on, restores
  recursive header install, or leaks EnTT/Taskflow/spdlog dependency discovery
  outside the static-link-only exception. The local-install package smoke
  `pixi run check-dart7-promotion-installed-package` verifies the same
  boundary after installation by compiling a downstream CMake project and
  checking that selected internal headers are absent from the installed prefix.
  `pixi run check-dart7-world-promotion-blockers` keeps the experimental
  namespace, staged package target, and build-option references in named
  transition buckets and fails if the
  code/build/test debt grows or main-tree parity references spread to new
  files; strict-final mode should only pass when the main branch no longer
  depends on the DART 6 simulation pipeline for the promoted API or parity
  evidence. `pixi run check-dart7-final-world-promotion` adds the required
  local `release-6.*` branch-ref check for final parity claims.

### Internal API

Internal API may change without compatibility guarantees. It includes:

- `detail/` and `internal/` directories.
- `dart::...::detail` namespaces.
- `*-impl.hpp` files, and existing `*_impl.hpp` files, that support templates
  or inline definitions.
- Private ECS/storage components such as `dart/simulation/comps/`.
- Symbols marked with a component `*_LOCAL` macro.
- Source-file-local classes, anonymous namespace helpers, and backend adapters.

Internal headers may be installed today for template and legacy reasons, but
that does not make the names supported user API.

If an internal type is already exposed through a public signature, track it as
exposed implementation compatibility debt before changing it.

## Symbol Visibility

Use component export macros only for symbols that are intentionally reachable
outside the shared library:

- Core library: `DART_API` for public exported symbols and `DART_LOCAL` for
  library-local symbols.
- GUI library: `DART_GUI_API` and `DART_GUI_LOCAL`.
- IO library: `DART_IO_API` and `DART_IO_LOCAL`.
- Other components follow the same `*_API` and `*_LOCAL` pattern.

`DART_STRICT_SYMBOL_VISIBILITY=ON` is an opt-in migration aid. It builds shared
libraries with hidden default visibility so missing export annotations are
easier to find. Keep it off for normal builds until the exported symbol set is
small and intentional.

Visibility is necessary but not sufficient: a hidden symbol can still leak
through headers, templates, Doxygen, or Python bindings.

## Header and Folder Rules

- Put new user-facing headers in the module root, using snake_case names.
- Put implementation details under `detail/` or `internal/`.
- Put template bodies in `detail/<name>-impl.hpp` only when the public template
  requires them; do not document those names as public API.
- Public headers should not require users to include backend-specific detail
  headers unless the backend type is itself a supported extension point.
- Prefer small value types, non-owning views, and explicit lifetime rules over
  exposing mutable implementation containers.

## Python Binding Rules

`dartpy` is a public API filter, not a mirror of every C++ symbol. New bindings
should expose the easiest supported workflow and avoid C++ internals:

- Do not include `dart/**/detail/**`, `dart/**/internal/**`, or
  `dart/simulation/comps/**` from `python/dartpy`.
- Do not bind `dart::...::detail` or `dart::...::internal` types, aliases, or
  data members. A public C++ alias that resolves to a detail type still needs
  migration tracking.
- Do not expose deprecated compatibility shims unless Python users need a
  migration window, and then add a deprecation warning, test, stub/doc update,
  and removal condition.
- Prefer Pythonic value wrappers or explicit public accessors when the C++ API
  still stores data in an internal base class.

The lint task `pixi run lint-api-boundaries` enforces the current Python
boundary, validates Doxygen boundary filters, audits generated C++ API docs when
they exist locally, and reports any legacy exceptions from
`scripts/check_api_boundaries_allowlist.txt`. Allowlist entries require
`replacement`, `remove_by`, `tracking`, and `reason` fields, and stale entries
fail the checker.

For scalable-compute work, `pixi run check-compute-backend-boundaries` also
checks public DART 7 simulation headers and the default dartpy simulation
bindings for CUDA/SYCL/device/stream/kernel/memory-pool API leakage. The
existing `ComputeStageAcceleration::Gpu` metadata flag is allowed because it
describes backend-neutral acceleration intent, not a concrete backend API.

## Documentation Rules

The public C++ API reference should describe public and compatibility APIs, not
implementation mechanics. Mark implementation-only comments with `@internal`
when they must live in a public header. Keep user guides focused on stable
workflows and link migration notes for deprecated compatibility APIs.

## Review Checklist

- Is this new symbol meant for users, downstream integrators, or only DART
  internals?
- If public, is it documented, tested, exported when needed, and represented in
  Python when appropriate?
- If compatibility-only, is the downstream reason clear and is the replacement
  documented with first deprecated release, earliest removal release, blocking
  migration condition, and changelog requirement?
- If it exposes a `detail` or `internal` type through public C++ or Python API,
  is it tracked as exposed implementation compatibility debt with replacement
  guidance?
- If internal, is it under `detail/` or `internal/`, hidden from `dartpy`, and
  excluded from generated user docs where practical?
- If it affects gz-physics, has the Gazebo integration workflow been run or
  explicitly deferred with a reason?
