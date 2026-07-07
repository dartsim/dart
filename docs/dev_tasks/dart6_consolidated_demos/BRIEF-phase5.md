# Phase 5 Brief: Retire Scattered Examples

Retire the individual `examples/*` and `python/examples/*` programs now
superseded by `dart-demos` (31 scenes) and `py-demos` (9 scenes). Read
`EVIDENCE-cleanup-refs.md` FIRST — it is the per-example reference table and
the source of truth for what breaks. Scope: this is a deletion + rewiring PR;
no new behavior. Branch: `feature/dart6-consolidated-demos`.

## Retire (folded into demos/py-demos)

C++ (`examples/`): add_delete_skels, atlas_puppet, atlas_simbicon,
biped_stand, box_stacking, boxes, contact_inverse_dynamics,
cylindrical_constraint*, drag_and_drop, dynamic_joint_constraints, empty,
fetch, hardcoded_design, heightmap, hubo_puppet, human_joint_limits,
hybrid_dynamics, imgui, joint_constraints, mixed_chain,
operational_space_control, point_cloud, rigid_chain, rigid_cubes, rigid_loop,
rigid_shapes, simple_frames, simulation_event_handler, sleeping, soft_bodies,
ssik_ik_gui, tinkertoy, vehicle, wam_ikfast.
(*cylindrical_constraint is console-only and NOT a demos scene — decide:
keep as a specialized standalone like hello_world, OR drop. Recommend KEEP
as a minimal non-GUI API example. Confirm in report.)

Python (`python/examples/`): atlas_puppet, biped_stand, contacts_pointcloud,
drag_and_drop, hello_world_gui, operational_space_control, rigid_chain,
rigid_cubes, rigid_loop, ssik_analytical_ik.

Delete the orphan stub `examples/rerun` (no sources, unregistered).

## KEEP standalone (do NOT retire)

- `examples/hello_world` (C++) + `python/examples/hello_world` — canonical
  minimal "first program" + installed-DART consumer sample.
- `examples/contact_benchmark` — CI-load-bearing (see below).
- `examples/speed_test` — console benchmark, belongs with perf work.
- `cylindrical_constraint` per recommendation above.

## Mandatory rewiring (same PR — from EVIDENCE-cleanup-refs.md)

1. `examples/CMakeLists.txt` — remove `add_subdirectory(<retired>)` lines;
   keep the file valid (MSVC builds examples by default — a broken file
   breaks the Windows default build).
2. `python/examples/CMakeLists.txt` — same for retired python dirs; ensure
   the `dartpy_add_example` glob still resolves; ADD a `demos` registration
   so `py-demos` is runnable (mirror how py examples register; the package
   uses `__main__.py`, so `python -m examples.demos`).
3. `pixi.toml` — remove/repoint dev tasks naming retired examples
   (`ex-atlas-puppet`, `ex-atlas-simbicon`, `py-atlas-puppet`,
   `py-ex-hello-world-gui`, `install-ssik` comment, etc.); ADD `demos` and
   `py-demos` run tasks (C++: `scripts/run_cpp_example.py`-style or direct
   `bin/dart-demos`; py: `python -m examples.demos` with the dartpy
   PYTHONPATH). Keep `ex`/`py-ex` generic defaults valid (they default to
   hello_world — retained).
4. `tools/freebsd/patches/patch-examples_wam__ikfast_Helpers.cpp` — delete
   or repoint (its target `examples/wam_ikfast/Helpers.cpp` is being
   removed; the ikfast code now lives under `examples/demos/scenes/
   wam_ikfast/ikfast/`).
5. `docs/readthedocs/dartpy/user_guide/examples.rst` — repoint the
   `ssik_analytical_ik`/`ssik_ik_gui` live commands at demos/py-demos.
6. `docs/readthedocs/gallery.rst` (+ ko `.po`) — the atlas_simbicon/tinkertoy
   pinned-commit links go stale; refresh wording to point at demos.
7. `examples/README.md` + `python/examples/README.md` — rewrite to describe
   the consolidated demos + the retained specialized examples.

## Do NOT touch

- `tests/benchmark/integration/` (pulls `examples/contact_benchmark` headers)
  and `.github/workflows/performance_dashboard_dart6.yml` (path trigger) —
  contact_benchmark stays, so these are untouched.
- Any `dart/` library code, public headers, package components.

## Verification

- `pixi run config` reconfigures cleanly (no missing add_subdirectory dirs).
- `pixi run build` (or at least `--target dart-demos` + `examples` aggregate)
  green; MSVC-path sanity: `examples/CMakeLists.txt` parses with all retired
  dirs gone.
- `rg` sweep proving no dangling references to any retired example remain
  outside its own (now-deleted) dir and outside docs/dev_tasks: rerun the
  EVIDENCE-cleanup-refs.md searches, expect zero hits for retired names in
  tests/CI/tutorials/scripts/pixi/CMake.
- `pixi run lint` green.
- `dart-demos --cycle-scenes` and `py-demos --cycle` still pass (demos are
  the replacement — prove they survive the example removal).
- Gazebo gate NOT required (examples are not package surface; no dart/
  changes) — but state that reasoning in the PR.

## Commit shape

One deletion commit (`git rm -r` the retired dirs) + the rewiring in the same
or an adjacent commit. Message: "Retire scattered examples superseded by
consolidated demos". No AI attribution. Do NOT push.
