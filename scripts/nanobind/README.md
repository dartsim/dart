# nanobind migration tools

Select a built `dartpy` with `PYTHONPATH` before running runtime probes. The
surface snapshot imports every public pure-Python subpackage first, so its
result does not depend on what was imported before it.
Snapshot runtime identities attribute inherited members to their walked defining class, use `identity_path: null` for members with unwalked owners, and match bound classmethod aliases by function and receiver. The separate `owner_path` records the defining class for budget attribution when unrelated classes export the same object. Members of a class are recorded under the first path the walk reaches it by, so a nested class reached first through an alias may have no rows at `<identity_path>.member`, and a class reached first at module level keeps its member rows there even when a defining class's path becomes its `identity_path`.

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
  enum values, properties, and descriptors. Each exported name counts once where
  it is exported, so aliases and same-name re-exports into another module count
  there; class members count on their defining class, so inherited members are
  not counted twice. An identical callable or property exported by unrelated
  classes counts in each class. The JSON key `containers` is a
  sorted list of the public exported container names per module -- classes,
  enums, and submodules -- per exported path and not deduplicated by
  identity, so an alias or re-export in a second module is listed in that
  module; nested classes count as members of their defining class instead.
  `container_owners` maps these names to canonical owners, so removing an owner
  also removes all its aliases while absent optional owners keep theirs.
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
  duplicates; privacy follows exported paths, so a publicly exported class with
  a private implementation name is tracked. Ordinary private and untracked
  external members are excluded; the snapshot's tracked special methods
  (`__init__`, `__GUIEventHandler__`, `__str__`, `__repr__`, `__mul__`, `__eq__`)
  count in the budget too. Enums count
  their declared names, excluding Python-version-dependent enum machinery.
  Class constants, exported enum values, and descriptors count only at the first
  class in the MRO whose `__dict__` defines the name, including when subclasses
  are visited first. Data names are not deduplicated by value identity.
  Only owners that build options guard (`OPTIONAL_OWNER_GROUPS` in `api_surface.py`:
  Bullet, ODE and Octomap classes, the model-asset retriever and the viewer) may
  be absent, and only when their entire feature group is absent. Bullet and ODE
  each guard a detector/group pair; a partially missing pair fails staleness.
  Any other missing budgeted owner fails, and `--update-budget`
  removes it from the budget. Absent optional owners and their exported container
  names are ignored by the test and retained by the updater; `--update-budget
  --prune` deletes those absent allowances too. Legacy budgets without
  `container_owners` retain module-wide container allowances when optional
  owners are absent until an update on a full-feature build records ownership.
  For present
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
- `pixi run dartpy scripts/nanobind/api_surface.py --check-budget` runs the
  ratchet, the R57 caps and the four R23 naming checks. The binding test
  `test_api_surface_budget` runs the same checks. "API budget checks" below
  describes them.
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

World is pinned to today's surface; R57 sets its DART 7 cap at 24 visible
public names (see `docs/design/dart7_python_naming.md`).

## API budget checks

R57 counts the visible public names of each class:

- its own and inherited public names, without dunders;
- each collection, sub-object or nested type once, under its exported name.

The existing counters keep their defining-owner attribution and their tracked
special methods.

Caps:

- The budget's top-level `caps` key maps a canonical class path to a limit.
- The check fails for a class over its cap and for a capped class that is
  missing.
- `--update-budget` and `--update-budget --prune` preserve the caps.
  `--allow-growth` cannot override a cap.
- Transform, Rotation and WorldState have a cap of 15. Later area packets add
  caps when their classes fit R57. World's cap of 24 waits until its legacy
  members leave.

The R23 naming checks:

1. A `get_` method needs a matching property.
2. No method that returns a value starts with `compute_`, `calc_` or `eval_`.
3. A `compute_` member is a method that returns `None`, and no keyword starts
   with `compute_`.
4. A noun has one kind across the public classes. Sequence `index` and `count`
   methods are protocol exceptions.

R2 defers GUI members, viewer members whose signatures use GUI types, and
aspect and composite state.

Known violations live in `python/tests/unit/bindings/api_naming_exceptions.json`.
Each entry names the check, the full member path, the rule, the reason and a
recorded exception reference: a section of a repository document, or the PR
that records it. Remove an entry when its member migrates. Add one only after
you record a rule exception, with its reason and the condition that ends it.

Freeze mode:

- camelCase and facade members coexist during the migration, so freeze mode is
  off by default.
- The API-freeze packet turns it on. It runs `--check-budget --freeze` and
  passes `freeze=True` to `naming_failures` in `test_api_surface_budget`.
- Freeze mode fails for any class outside R2 that has a nonzero `camel_case`
  counter, whatever the known exceptions say.

## Recorded naming exceptions

R23 temporarily allows these provisional lookups without matching properties:

- `dartpy.dynamics.SoftBodyNode.get_face`
- `dartpy.dynamics.SoftBodyNode.get_point_mass`

The soft-body area replaces them with R27 collections and removes these
exceptions. Coexistence keeps today's callers working until that migration.

R59 has these temporary exceptions while R41 retains the legacy math types:

- `dartpy.math.Isometry3.matrix`
- `dartpy.math.Isometry3.rotation`
- `dartpy.math.Isometry3.translation`
- `dartpy.math.AngleAxis.rotation`
- `dartpy.math.Quaternion.rotation`
- `dartpy.math.Quaternion.wxyz`

These callables coexist with the same nouns as Transform and Rotation
properties. Removing each legacy type with its last consumer ends its
exceptions. No new class or member inherits an exception merely by using
one of these nouns.
