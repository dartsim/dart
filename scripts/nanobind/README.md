# nanobind migration tools

Select a built `dartpy` with `PYTHONPATH` before running runtime probes.
Snapshot runtime identities attribute inherited members to their walked defining class, use `identity_path: null` for members with unwalked owners, and match bound classmethod aliases by function and receiver; members of a class are recorded under the first path the walk reaches it by, so a nested class reached first through an alias may have no rows at `<identity_path>.member`, and a class reached first at module level keeps its member rows there even when a defining class's path becomes its `identity_path`.

- `api_surface.py --pb <pybind11-module-dir> --nb <nanobind-module-dir>
  --output <output-dir>` snapshots and compares the public API, including `dartpy.gui`: names, overloads, keywords, defaults, and enums.
- `pixi run dartpy scripts/nanobind/api_surface.py --update-budget` lowers the
  checked-in `python/tests/unit/bindings/api_budget.json` counts to the live
  module's surface. Run updates on a full-feature build, including Bullet, ODE,
  and Octomap. Adding `--allow-growth` explicitly permits increases (also
  required to create a budget) and prints every raised counter. Review the
  resulting diff. The normal binding tests reject increases and stale ceilings
  in public, camelCase/C++ `m_` names, or `get`/`set` accessor names, per class
  and per module for `functions` and `classes`. The JSON key `functions`
  includes every non-container module member: callables, constants, exported
  enum values, properties, and descriptors. The JSON key `containers` is a
  sorted list of the public exported container names per module -- classes,
  enums, and submodules -- per exported path and not deduplicated by
  identity, so an alias or re-export in a second module is listed in that
  module; nested classes count as members of their defining class instead.
  Any current container name missing from the budget fails as growth on every
  build, with the name in the message. A new module or class that appears
  with empty counters is also reported as growth.
  Accessors match `^(?i:get|set)([A-Z_]|$)`: the verb is case-insensitive, but
  the following capital-or-underscore boundary is case-sensitive. `settings`
  and `setup` do not count; `setUpView` counts as a setter verb under this naming
  heuristic. A single-word PascalCase name such as `Step` counts only in
  `public`, not `camel_case`; replacing `step` with `Step` leaves all counts
  unchanged, so the counts-only ratchet cannot detect that rename.
  Canonical module
  and class names and snapshot object identities avoid re-export and inherited
  duplicates; private and untracked external members are excluded. Enums count
  their declared names, excluding Python-version-dependent enum machinery.
  Class constants, exported enum values, and descriptors count only at the first
  class in the MRO whose `__dict__` defines the name, including when subclasses
  are visited first. Data names are not deduplicated by value identity.
  Absent optional modules, classes, and submodules are ignored by the test and
  retained by the updater: a module that lacks any budgeted class or submodule
  skips container staleness, and `--update-budget` without `--prune` keeps the
  budgeted container names of such modules; only `--update-budget --prune`
  rewrites container lists from the build and deletes absent owners, so after
  removing or renaming a class, run it on a full-feature build. For present
  owners and complete modules, removals require lowering budgets with
  `--update-budget`, without `--allow-growth`. Growth requires reverting the
  change or deliberately raising budgets with `--update-budget --allow-growth`.
  Failure diagnostics for `functions` and `classes` list the count and sorted
  current names for each exceeded counter, including additions, truncated to 40
  names with `... (N more)`: counts alone cannot identify which specific names
  are new. Container failures name the new or missing container names directly.
  With `PYTHONPATH` already
  selecting a build, `pixi run python scripts/nanobind/api_surface.py
  --update-budget` also works.
- `probe_gui_rss.py` checks 1,000 GUI owner lifecycles without opening a window.
- `visual_scenes.py` supplies the existing dominoes tutorial to `agent-capture`.
- `codemod.py --source <DART-6.20-pybind11-sources> --output <new-directory>`
  generates a mechanical non-GUI port for review from an explicit historical
  source tree. It refuses an existing destination. Review the caster includes,
  ownership policies, and constructors before using the generated bindings.
  Select `detail/eigen.hpp`, `detail/array.hpp`, and the needed STL casters,
  then use `check_guards.py` to check the include contract.
- `probe_gc.py` checks the eight supported ownership-cycle shapes;
  `probe_gc_clear.py` exercises both clear orders and idempotent clearing of the
  nanobind binder's native-owner GC slots.
- `probe_gc_aliases.py` reports conservative collection with a shared Python pin
  and verifies release after explicit cleanup.
- `probe_secondary_inputs.py` checks non-polymorphic secondary-base inputs.
- `probe_properties.py` inventories default-constructible property getters;
  `nullable_inventory.py` inventories nullable nanobind signatures.
- `probe_eigen.py --build-dir <new-build-directory>` builds a small extension
  using the production caster and checks NumPy runtime selection, Eigen export
  lifetime, and mutable vector inputs. Its docstring includes NumPy 1.x and 2.x
  Pixi invocations.

The regular regressions live in `python/tests/unit/bindings/` and run with
nanobind. The `--pb` API comparison input is a separately built DART 6.20 module;
this tree no longer contains its binder. Compile/import measurements and
research reports are not part of these tools.

World is pinned to today's surface; the DART 7 target is about 20 public names
(see `docs/design/dart7_python_api.md`, "World, modes and mutation classes").
