# DART 7 `dart::simulation::World` Name-Collision Transaction

Status: Proposed (needs maintainer sign-off).

This design owns the durable rationale for **how the classic DART 6
`dart::simulation::World` name is cleared** so the ECS-backed experimental world
can own the official DART 7 `dart::simulation::World` C++ name and the
`dartpy.World` / `dartpy.simulation.World` Python names. It is the design
artifact for PLAN-041 workstream 4 (World name-collision transaction). Roadmap
operating state for PLAN-041 lives in `docs/plans/dashboard.md`; the active
workstream sequence, verification commands, and acceptance criteria live in
`docs/plans/041-official-simulation-api-promotion.md`. The promoted C++ and
Python API shapes are owned by `docs/design/simulation_experimental_cpp_api.md`
and `docs/design/simulation_experimental_python_api.md`; the release topology is
owned by `docs/design/dart7_clean_break_strategy.md`. Public/internal API policy
is owned by `docs/onboarding/api-boundaries.md`. This doc only owns the
transaction that frees the colliding name without ever shipping two official
worlds.

## Why This Is A Hard Blocker

The name `dart::simulation::World` is currently occupied by the classic DART 6
world, and the ECS-backed world lives under `dart::simulation::experimental`.
Promotion cannot simply rename the experimental class into `dart::simulation`
because that name is already a fully exported, default-built, widely consumed
public symbol. The two definitions are not ABI- or lifecycle-compatible (see the
occupancy map below), so there is no safe in-place type alias. The collision
must be cleared by an explicit transaction before the official facade PR can
land.

## Current Occupancy Of `dart::simulation::World` (C++)

### Classic world (DART 6 surface, occupies the official name today)

- Class declaration: `dart/simulation/world.hpp:139` —
  `class DART_API World : public virtual common::Subject`, namespace
  `dart::simulation` (`dart/simulation/world.hpp:71-72`).
- Forward declaration and smart-pointer typedefs:
  `dart/simulation/fwd.hpp:42-44` (`class World;` +
  `DART_COMMON_DECLARE_SHARED_WEAK(World)`), so `dart::simulation::WorldPtr`,
  `WeakWorldPtr`, etc. are part of the public name set.
- Lifecycle: heap-allocated shared ownership. Public factories
  `static WorldPtr create(...)` (`dart/simulation/world.hpp:147-158`),
  `std::shared_ptr<World> clone() const`
  (`dart/simulation/world.hpp:171`), `explicit World(const WorldConfig& config)`
  (`dart/simulation/world.hpp:164`), and the companion value type
  `struct WorldConfig final` (`dart/simulation/world.hpp:108`). It derives from
  `common::Subject` (observer pattern) and manages skeletons/frames through
  `common::NameManager` (`dart/simulation/world.cpp:226-227`,
  `dart/simulation/world.cpp:484-524`).
- Export macro: `DART_API` (resolved in `dart/export.hpp:37-41`).
- Build target/component: built into the **core `dart` target**. `dart/simulation`
  is added unconditionally via `add_subdirectory(simulation)`
  (`dart/CMakeLists.txt:93`); `dart/simulation/CMakeLists.txt:6-7` folds these
  headers/sources into `dart_add_core_headers` / `dart_add_core_sources`. The
  classic world is therefore **present in every default build** with no opt-in
  flag.
- Installed headers: `dart/simulation/world.hpp` installs to
  `include/dart/simulation/world.hpp` (`dart/simulation/CMakeLists.txt:22-29`),
  plus generated `All.hpp` / `simulation.hpp` component headers and PascalCase
  compatibility headers (`dart/simulation/CMakeLists.txt:9-39`).
- In-tree consumers: grepping C/C++ source and header files for
  `simulation::World` or `dart/simulation/world.hpp` currently finds 158 files:
  35 under `dart/`, 1 under `dartsim/`, 47 under `examples/`, 4 under
  `python/`, and 71 under `tests/`. The runtime/library subset includes
  `dart/utils/**` parsers (URDF/SDF/MJCF/SKEL), `dart/io/read.hpp`,
  `dart/gui/**`, `dart/sensor/**`, and `dartsim/ui/src/editor.cpp`; examples,
  tests, and Python bindings are also part of the in-repo migration surface.

### ECS-backed world (experimental, target for promotion)

- Class declaration: `dart/simulation/experimental/world.hpp:130` —
  `class DART_EXPERIMENTAL_API World`, namespace
  `dart::simulation::experimental` (`dart/simulation/experimental/world.hpp:58`).
- Lifecycle: value type, **non-copyable and non-movable** (all copy/move
  constructors and assignments deleted,
  `dart/simulation/experimental/world.hpp:142-145`); stack-constructible
  (`World()` and `explicit World(const WorldOptions&)`,
  `dart/simulation/experimental/world.hpp:133-138`). No `common::Subject` base,
  no `WorldPtr` shared-ownership typedef.
- Internal/ECS exposure today: `entt::registry& getRegistry()` and the const
  overload (`dart/simulation/experimental/world.hpp:449-452`, marked
  `@internal` as a DART 7 escape hatch), backed by the data member
  `entt::registry m_registry` (`dart/simulation/experimental/world.hpp:513`).
  The header includes `<entt/entt.hpp>`
  (`dart/simulation/experimental/world.hpp:47`). These EnTT/ECS dependencies are
  exactly what the promoted boundary checks must reject (PLAN-041 workstream 3),
  so a wrapper/opaque-impl facade is required regardless of which transaction
  option is chosen.
- Export macro: `DART_EXPERIMENTAL_API` (distinct from `DART_API`).
- Build target/component: separate CMake component `simulation-experimental`,
  target `dart-simulation-experimental`
  (`dart/simulation/experimental/CMakeLists.txt:31-32`,
  `dart/simulation/experimental/CMakeLists.txt:197-200`). It links
  `EnTT::EnTT` **PUBLIC** (`dart/simulation/experimental/CMakeLists.txt:115`) and
  is controlled by `DART_BUILD_SIMULATION_EXPERIMENTAL`, which defaults to **OFF**
  (`CMakeLists.txt:409-414`, `CMakeLists.txt:897-899`). The header style is
  `#pragma once` and the headers install via a `DIRECTORY` glob to
  `include/dart/simulation/experimental`
  (`dart/simulation/experimental/CMakeLists.txt:202-208`); they do **not** flow
  through `dart_generate_component_headers`, so no PascalCase compat header is
  generated for the experimental world today.

The classic world is non-optional and core; the ECS world is opt-in and
EnTT-coupled. PLAN-041 workstream 2 requires the official baseline to be
non-optional and EnTT/ECS-free at the public boundary before promotion is
claimed, so the build-shape change is a prerequisite of either transaction
option.

## Current Occupancy Of `dartpy.World` And `dartpy.simulation.World` (Python)

- Classic binding: `nb::class_<World>(m, "World")` where
  `World = dart::simulation::World`, registered into the `simulation` submodule
  (`python/dartpy/simulation/world.cpp:32`,
  `python/dartpy/simulation/world.cpp:78`, via `defWorld(m)` from
  `python/dartpy/simulation/module.cpp:10`, wired by
  `m.def_submodule("simulation", ...)` and `defSimulationModule` at
  `python/dartpy/dartpy.cpp:86-88`). Result: `dartpy.simulation.World` is the
  classic world.
- Experimental binding: `nb::class_<sim::World>(m, "World")` where
  `sim = dart::simulation::experimental`
  (`python/dartpy/simulation_experimental/module.cpp:102`,
  `python/dartpy/simulation_experimental/module.cpp:2212`), registered into the
  `simulation_experimental` submodule
  (`python/dartpy/dartpy.cpp:91-94`). Result:
  `dartpy.simulation_experimental.World` is the ECS world.
- Both bindings use the nanobind class name `"World"`, but in **different
  submodules**, so there is no duplicate-registration conflict today. nanobind
  keys class identity by C++ type within a module scope; binding both C++ types
  with the name `"World"` into the **same** `simulation` module would be a
  duplicate-registration hazard. This constrains the atomic-replacement option:
  only one C++ type may own `simulation.World` at a time.
- Top-level `dartpy.World`: there is no explicit re-export in
  `python/dartpy/__init__.py`. The top-level name is synthesized at import time
  by `_layout.install_layout` →
  `_promote_symbols(root, _PROMOTE_MODULES)` (`python/dartpy/_layout.py:36-38`,
  `python/dartpy/_layout.py:65-81`, `python/dartpy/_layout.py:127-131`).
  `_PROMOTE_MODULES` includes `"simulation"`, so the loader copies
  `dartpy.simulation.World` onto the top-level package as `dartpy.World`.
  Therefore `dartpy.World` **currently resolves to the classic world** and
  silently tracks whatever `dartpy.simulation.World` is.
- Stubs: `python/stubs/dartpy/simulation.pyi:90` declares `class World:` (the
  classic shape, with `create`/`clone`/`WorldConfig`), and
  `python/stubs/dartpy/__init__.pyi:256-268` re-exports `World` from
  `.simulation` (and lists it in `__all__` at line 402). The experimental world
  is a separate `class World:` in
  `python/stubs/dartpy/simulation_experimental.pyi:1560`. Any change to which
  C++ type backs `simulation.World` must regenerate both stub files
  (`pixi run generate-stubs`) and the generated Python API docs
  (`pixi run api-docs-py`).

## The Case-Insensitive `World.hpp` / `world.hpp` Filesystem Collision

`dart/simulation/CMakeLists.txt:9-18` runs the classic simulation headers
through `dart_generate_component_headers`, which auto-generates PascalCase
compatibility headers for snake_case headers via `dart_snake_to_pascal`
(`cmake/dart_defs.cmake:1140-1183`, `cmake/dart_defs.cmake:1285`).
`dart_snake_to_pascal("world.hpp")` yields `World.hpp`. On a **case-sensitive**
filesystem (typical Linux CI), `dart/simulation/World.hpp` is generated and
installed alongside the source `dart/simulation/world.hpp` with no conflict. On
a **case-insensitive** filesystem (default macOS APFS, Windows NTFS),
`World.hpp` and `world.hpp` are the same path and would collide.

The generator already guards this: on a detected case-insensitive filesystem it
**skips** single-word snake_case headers that contain no underscore
(`cmake/dart_defs.cmake:1276-1282`, the `continue()` branch). `world.hpp` has no
underscore, so on case-insensitive filesystems no `World.hpp` compat header is
emitted, avoiding the clash.

Transaction implications:

- If the promoted ECS facade keeps the public header name
  `dart/simulation/world.hpp` (recommended for source compatibility of the
  include path), the existing guard keeps the build correct on all filesystems;
  the PascalCase `World.hpp` compat header simply does not exist on
  case-insensitive filesystems and exists harmlessly on case-sensitive ones.
- A migration that introduces a _new_ official header with an underscore-free
  PascalCase-colliding name (for example a temporary `dart/simulation/world2.hpp`
  is fine; a `dart/simulation/World.hpp` hand-written facade is not) must respect
  this guard. Do not hand-author a `dart/simulation/World.hpp`; it would collide
  with the generated compat header on case-sensitive filesystems and with the
  source `world.hpp` on case-insensitive ones.
- Whichever option is chosen, validate the facade on both case-sensitive and
  case-insensitive filesystems so the `World.hpp`/`world.hpp` behavior is
  exercised before promotion.

## Option (a): Temporary Non-Colliding Facade While Consumers Migrate

Introduce the promoted ECS-backed world under a temporary, non-colliding public
name (for example `dart::simulation::World7` / `dart/simulation/world7.hpp`, or a
nested `dart::simulation::v7::World`), keep the classic
`dart::simulation::World` available for one migration window, port consumers,
then in a later PR swap the temporary name onto `dart::simulation::World` and
remove the classic definition.

Pros:

- Smallest blast radius per PR. The first facade PR adds a new symbol and breaks
  nothing; the 159 in-tree consumers migrate incrementally behind a green build.
- Lets parity, boundary, and package validation stabilize against the new facade
  before the classic world is removed, reducing the chance of a single giant PR
  failing late.
- Python mirror is clean: bind the new world into a non-colliding submodule
  (or a `simulation.World7` class name) so there is never duplicate
  registration of `"World"` in the same module; `dartpy.World` keeps pointing at
  the classic world until the final swap.

Cons:

- Ships **two worlds at once** during the window, which is exactly what PLAN-041
  acceptance criteria warn against ("This avoids shipping two official worlds").
  The temporary name must be explicitly marked non-official, time-boxed, and
  verified, or it risks becoming a de facto second public API.
- A throwaway public name (`World7`, `v7::World`) leaks transition mechanics into
  user-facing code and downstream (gz-physics) include paths, then forces a
  second rename. Two renames cost more total churn than one.
- ABI: introduces and then removes a public symbol within the DART 7 line,
  producing two ABI breaks instead of one.
- Higher coordination cost: the final swap PR still has to atomically retarget
  the name, so the hard step is deferred, not avoided.

## Option (b): Atomic Replacement In One PR

In a single PR, replace the classic `dart::simulation::World` definition with the
ECS-backed facade (public wrapper that hides EnTT/ECS), update
`dart/simulation/fwd.hpp`, retarget all in-tree consumers, repoint the Python
`simulation.World` binding to the ECS facade, and quarantine the classic
implementation under an explicit legacy/internal namespace/target that **no
longer exports `dart::simulation::World`** (or remove it outright). The facade
preceding work (build-shape change to make the baseline non-optional and
EnTT-free at the boundary, PLAN-041 workstream 2/5) lands first.

Pros:

- Never ships two official `dart::simulation::World` definitions; the public name
  means exactly one world at every committed state, matching the PLAN-041
  acceptance criteria directly.
- One rename, one ABI break, one downstream (gz-physics) migration event. The
  official include path `dart/simulation/world.hpp` and namespace
  `dart::simulation::World` are preserved, so source-compatible consumers that
  use only the overlapping API need minimal churn.
- No throwaway public name leaks into user or downstream code.
- Python side is unambiguous: `simulation.World` is rebound to the ECS facade in
  the same PR, `dartpy.World` follows automatically via `_promote_symbols`, and
  stubs/docs regenerate once.

Cons:

- Large, high-risk PR touching 158 in-tree files across runtime code, examples,
  tests, and Python bindings, plus stubs and generated docs. PLAN-041 should
  sequence prerequisite validation so the final replacement is not the first time
  lifecycle, package, Python, and boundary decisions meet.
- The ECS world is non-copyable/non-movable and value-typed, while the classic
  world is shared-ownership (`WorldPtr`, `clone()`, `create()`). The facade must
  reconcile lifecycle: either the facade re-adds heap/shared-ownership factories
  on top of the ECS world, or consumers move to value/owning semantics in the
  same PR. This is the dominant correctness risk and cannot be a literal type
  alias.
- nanobind duplicate-registration must be handled in the cutover: the classic
  `nb::class_<dart::simulation::World>(m, "World")` must be removed from the
  `simulation` module in the same change that adds the ECS binding there, so the
  name `"World"` in the `simulation` module is never bound to two C++ types.
- Heavier merge-conflict exposure for active solver/loader branches at the moment
  the PR lands (PLAN-041 calls for `git merge-tree` preflight against active PR
  heads).

## Recommended Transaction

**Adopt option (b), atomic replacement, with prerequisites sequenced in
PLAN-041 before the replacement PR.** Option (b) is the only path that never
ships two official `dart::simulation::World` symbols, costs one rename and one
ABI/downstream migration, and preserves the official include path and namespace.
The two-worlds window and double rename of option (a) directly conflict with the
PLAN-041 acceptance criterion of not shipping two official worlds, and the
throwaway public name would leak into gz-physics include paths.

The durable design decision is the ownership model for the public name: there is
exactly one official `dart::simulation::World` at each committed state, and the
classic implementation is either removed or quarantined somewhere that no longer
exports that name. PLAN-041 owns the active PR sequence, verification commands,
and open maintainer decisions needed to make the atomic replacement reviewable.

## `dartpy.World` Decision

Options:

- **Remove** `dartpy.World` (top-level) and require `dartpy.simulation.World`.
- **Deprecate**: keep `dartpy.World` as a warning-emitting alias of
  `dartpy.simulation.World` for one window, then remove.
- **Alias / identical**: make `dartpy.World` permanently identical to
  `dartpy.simulation.World` (same class object), which is the current de facto
  behavior produced by `_layout._promote_symbols`.

Recommendation: **keep `dartpy.World` identical to `dartpy.simulation.World`**,
because the existing `_promote_symbols` machinery
(`python/dartpy/_layout.py:65-81`) already makes the top-level name an exact
copy of the submodule class object, and the DART 7 layout intentionally promotes
flat top-level names. After the Python cutover, `dartpy.simulation.World` is the
ECS facade, so `dartpy.World` automatically
becomes the ECS facade with the same class identity — `isinstance` and `is`
checks stay consistent across `dartpy.World` and `dartpy.simulation.World`
because they are the same object, not two registrations.

Rationale and guardrails:

- The risk PLAN-041 flags is _silently_ repointing `dartpy.World` at a different
  world without a maintainer decision. Choosing "identical to
  `dartpy.simulation.World`" makes the repoint explicit and validated rather
  than silent: the same Python cutover that rebinds `simulation.World` updates
  `python/stubs/dartpy/__init__.pyi` (currently re-exporting `World` from
  `.simulation`, lines 256-268/402) and `simulation.pyi` (currently the classic
  shape, line 90), and adds a wheel import smoke asserting
  `dartpy.World is dartpy.simulation.World` and that the world is the ECS facade.
- This avoids a second public name (no top-level deprecation shim to maintain)
  and avoids duplicate nanobind registration, since `dartpy.World` is never an
  independent binding — only a promoted reference to the single
  `simulation.World` class object.
- If maintainers prefer a stricter clean break, the fallback is **deprecate**:
  keep `dartpy.World` as a `DeprecationWarning`-emitting alias of
  `dartpy.simulation.World` for one DART 7 minor, then remove. "Remove
  immediately" is not recommended for the first promoted release because many
  examples and downstream snippets use the flat `dartpy.World` name; removing it
  in the same release as the world swap doubles the migration cost for users.

Either chosen behavior must be covered by tests, regenerated stubs, regenerated
docs, and wheel import smokes per the PLAN-041 acceptance criteria.
