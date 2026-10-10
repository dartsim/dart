# nanobind migration tools

Select a built `dartpy` with `PYTHONPATH` before running runtime probes.

- `api_surface.py --pb <pybind11-module-dir> --nb <nanobind-module-dir>
  --output <output-dir>` snapshots and compares the public API, including `dartpy.gui`, including
  names, overloads, keywords, defaults, and enums.
- `probe_gui_rss.py` checks 1,000 GUI owner lifecycles without opening a window.
- `visual_scenes.py` supplies the existing dominoes tutorial to `agent-capture`.
- `codemod.py --output <new-directory>` generates a mechanical non-GUI port for
  review. It refuses an existing destination. Review the caster includes,
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

The regular regressions live in `python/tests/unit/bindings/` and run with either
binder. Compile/import measurements and research reports are not part of these
tools.
