# DART 7 `dart::simulation::World` Name-Collision Transaction

Status: Accepted and implemented on the DART 7 promotion branch. The DART 7
transaction uses option (b): atomic replacement of the classic
`dart::simulation::World` public name with the ECS-backed facade. Do not
introduce a temporary public `World7`, `simulation::v7::World`, or equivalent
two-world migration surface on `main`.

This design owns the durable rationale for **how the classic DART 6
`dart::simulation::World` name is cleared** so the ECS-backed World can own the
official DART 7 `dart::simulation::World` C++ name and the
`dartpy.World` / `dartpy.simulation.World` Python names. It is the design
artifact for PLAN-041 workstream 4 (World name-collision transaction). Roadmap
operating state for PLAN-041 lives in `docs/plans/dashboard.md`; the active
workstream sequence, verification commands, and acceptance criteria live in
`docs/plans/041-official-simulation-api-promotion.md`. The promoted C++ and
Python API shapes are owned by `docs/design/simulation_cpp_api.md`
and `docs/design/simulation_python_api.md`; the release topology is
owned by `docs/design/dart7_clean_break_strategy.md`. Public/internal API policy
is owned by `docs/onboarding/api-boundaries.md`. This doc only owns the
transaction that frees the colliding name without ever shipping two official
worlds.

## Why This Was A Hard Blocker

The name `dart::simulation::World` was occupied by the classic DART 6 world, and
the ECS-backed world lived under `dart::simulation::experimental`. Promotion
could not simply rename the staged class into `dart::simulation` because that
name was already a fully exported, default-built, widely consumed public symbol.
The two definitions were not ABI- or lifecycle-compatible (see the occupancy map
below), so there was no safe in-place type alias. The collision had to be
cleared by an explicit transaction before DART 7 could claim the official
facade.

## Pre-Transaction Occupancy Of `dart::simulation::World` (C++)

### Classic world (DART 6 surface, occupied the official name)

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
  `dart/io/**` parsers (URDF/SDF/MJCF), `dart/io/read.hpp`,
  `dart/gui/**`, `dart/sensor/**`, and `dartsim/ui/src/editor.cpp`; examples,
  tests, and Python bindings are also part of the in-repo migration surface.

### ECS-backed world (staging surface, target for promotion)

- Class declaration: `dart/simulation/experimental/world.hpp:231` —
  `class DART_EXPERIMENTAL_API World`, namespace
  `dart::simulation::experimental` (`dart/simulation/experimental/world.hpp:58`).
- Lifecycle: value type, **non-copyable and non-movable** (all copy/move
  constructors and assignments deleted around the public class declaration);
  stack-constructible through `World()` and
  `explicit World(const WorldOptions&)`. No `common::Subject` base, no
  `WorldPtr` shared-ownership typedef.
- Internal/ECS exposure state: the staged public header no longer exposes
  `entt::registry`, `getRegistry()`, or `<entt/entt.hpp>`. It owns opaque
  `std::unique_ptr<detail::WorldStorage> m_storage`
  (`dart/simulation/experimental/world.hpp:807`) and grants detail-only storage
  access through `detail::storageOf(...)`
  (`dart/simulation/experimental/world.hpp:72-74`,
  `dart/simulation/experimental/world.hpp:750-751`). This clears the staged
  public-header leak, but the final promoted facade still needs the official
  namespace, export macro, package target/component, and classic-World
  quarantine/removal transaction.
- Export macro: `DART_EXPERIMENTAL_API` (distinct from `DART_API`).
- Build target/component: separate CMake component `simulation-experimental`,
  target `dart-simulation-experimental`. It now keeps EnTT/Taskflow/spdlog as
  private implementation dependencies for shared builds, exposes static
  link-only dependency packages only under `if(NOT BUILD_SHARED_LIBS)`, and the
  root build attempts it whenever the required sibling targets are present.
  Headers install through the explicit `DART_EXPERIMENTAL_PUBLIC_HEADERS`
  allowlist, not a recursive
  directory glob; they still do **not** flow through
  `dart_generate_component_headers`, so no PascalCase compat header was
  generated for the staged DART 7 simulation world at that time.

The classic world is non-optional and core; the ECS world is staged behind an
experimental namespace/target, without a separate build option, and has a clean
staged public-header boundary. PLAN-041 workstream 2 requires the official
baseline to move that contract onto the final package shape before
promotion is claimed, so the build-shape change remains a prerequisite of either
transaction option.

## Python Occupancy Of `dartpy.World` And `dartpy.simulation.World`

- ECS binding: `nb::class_<sim::World>(m, "World")` where
  `sim = dart::simulation` lives in `python/dartpy/simulation/module.cpp` and
  registers into the `simulation` submodule. Result: `dartpy.simulation.World`
  is the ECS-backed DART 7 facade.
- Classic binding: the old `dart::simulation::World` binding is no longer
  exposed as `dartpy.simulation.World`. It is registered through
  `python/dartpy/simulation/world.cpp` as `dartpy.gui.RenderWorld`, with
  `WorldConfig`, collision/LCP enums, and `ConstraintSolver` retained only as
  rendering/parser plumbing. This keeps the Python public simulation API from
  shipping two `World` classes.
- `dartpy.simulation_experimental`: the runtime/stub layout no longer publishes
  this as a second submodule. If maintainers reintroduce a migration alias, it
  must be a Python-level alias forwarding to `dartpy.simulation`, not a second
  nanobind registration.
- Top-level `dartpy.World`: `_layout._PROMOTE_MODULES` promotes `simulation`
  first, so `dartpy.World` is the same class object as
  `dartpy.simulation.World`. The generated top-level stub re-exports `World`
  from `.simulation` and does not import `simulation_experimental`.
- Stubs and checks: `python/stubs/dartpy/simulation.pyi` declares the ECS
  `class World`, `python/stubs/dartpy/__init__.pyi` re-exports it, and
  `python/stubs/dartpy/simulation_experimental.pyi` is absent. Keep this guarded
  with `pixi run check-dartpy-import-layout`, `pixi run generate-stubs`, and
  `pixi run api-docs-py` when Python module names change.

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
- Python mirror is no longer a reason to choose this option: the Python side has
  already moved the ECS-backed World into `dartpy.simulation.World` /
  `dartpy.World` and quarantined the classic world as `dartpy.gui.RenderWorld`.

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
`dart/simulation/fwd.hpp`, retarget all in-tree C++ consumers, keep the already
promoted Python `simulation.World` / `dartpy.World` identity aligned with the
ECS facade, and quarantine the classic implementation under an explicit
legacy/internal namespace/target that **no longer exports
`dart::simulation::World`** (or remove it outright). The facade preceding work
(build-shape change to make the baseline non-optional and EnTT-free at the
boundary, PLAN-041 workstream 2/5) lands first.

Pros:

- Never ships two official `dart::simulation::World` definitions; the public name
  means exactly one world at every committed state, matching the PLAN-041
  acceptance criteria directly.
- One rename, one ABI break, one downstream (gz-physics) migration event. The
  official include path `dart/simulation/world.hpp` and namespace
  `dart::simulation::World` are preserved, so source-compatible consumers that
  use only the overlapping API need minimal churn.
- No throwaway public name leaks into user or downstream code.
- Python side is already unambiguous: `simulation.World` is the ECS facade,
  `dartpy.World` follows via `_promote_symbols`, and
  `check-dartpy-import-layout` / generated stubs guard that identity while the
  C++ transaction lands.

Cons:

- Large, high-risk PR touching 158 in-tree files across runtime code, examples,
  tests, and any Python/stub/docs references that still mention classic or
  experimental paths. PLAN-041 should sequence prerequisite validation so the
  final replacement is not the first time lifecycle, package, and boundary
  decisions meet.
- The ECS world is non-copyable/non-movable and value-typed, while the classic
  world is shared-ownership (`WorldPtr`, `clone()`, `create()`). The facade must
  reconcile lifecycle: either the facade re-adds heap/shared-ownership factories
  on top of the ECS world, or consumers move to value/owning semantics in the
  same PR. This is the dominant correctness risk and cannot be a literal type
  alias.
- nanobind duplicate-registration must be handled in the cutover: the classic
  `nb::class_<dart::simulation::World>(m, "World")` is already removed from the
  Python `simulation` module. Keep it that way: the name `"World"` in the
  `simulation` module must never be bound to two C++ types.
- Heavier merge-conflict exposure for active solver/loader branches at the moment
  the PR lands (PLAN-041 calls for `git merge-tree` preflight against active PR
  heads).

## Accepted Transaction

**Adopt option (b), atomic replacement, with prerequisites sequenced in
PLAN-041 before the replacement PR.** This follows the DART 7 clean-break
direction: DART 6 compatibility and parity evidence live on `release-6.*`
branches, while `main` moves to the new public API without preserving a second
official World facade. Option (b) is the only path that never
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
`check-dart7-world-promotion-blockers` keeps the blocker set executable:
default mode prevents new unclassified classic/experimental references, rejects
temporary `World7`/`simulation::v7::World` facade code, rejects growth in the
code/build/test buckets, and ratchets main-tree parity references to zero. The
contact/constraint World rows, skeleton-to-multibody multi-skeleton rows, and
former world-parity rows have been converted to DART 7-only regression coverage,
so reintroducing classic World references there fails the default ratchet. The
final gate is `pixi run check-dart7-final-world-promotion`, which also requires
a local `release-6.*` branch ref before parity can be claimed.

## `dartpy.World` Decision

Options:

- **Remove** `dartpy.World` (top-level) and require `dartpy.simulation.World`.
- **Deprecate**: keep `dartpy.World` as a warning-emitting alias of
  `dartpy.simulation.World` for one window, then remove.
- **Alias / identical**: make `dartpy.World` permanently identical to
  `dartpy.simulation.World` (same class object), which is the current de facto
  behavior produced by `_layout._promote_symbols`.

Current decision: **keep `dartpy.World` identical to
`dartpy.simulation.World`**,
because the existing `_promote_symbols` machinery
(`python/dartpy/_layout.py:65-81`) already makes the top-level name an exact
copy of the submodule class object, and the DART 7 layout intentionally promotes
flat top-level names. The Python cutover has happened:
`dartpy.simulation.World` is the ECS facade, so `dartpy.World` is the ECS facade
with the same class identity — `isinstance` and `is` checks stay consistent
across `dartpy.World` and `dartpy.simulation.World`
because they are the same object, not two registrations.

Rationale and guardrails:

- The risk PLAN-041 flags is _silently_ repointing `dartpy.World` at a different
  world without a maintainer decision. Choosing "identical to
  `dartpy.simulation.World`" makes the repoint explicit and validated rather
  than silent: the Python cutover updates `python/stubs/dartpy/__init__.pyi` and
  `simulation.pyi`, and `check-dartpy-import-layout` asserts that
  `dartpy.World is dartpy.simulation.World`, that
  `dartpy.simulation_experimental` is absent, and that the classic render world
  is only `dartpy.gui.RenderWorld`.
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
