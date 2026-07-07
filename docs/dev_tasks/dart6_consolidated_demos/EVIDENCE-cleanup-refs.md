# Evidence: External References Gating Example Retirement (Phase 5)

Collected 2026-07-04 by recon agent (rg sweep of .github, tests, tutorials,
docs, scripts, tools, pixi.toml, CMake, python/). Full table in transcript;
decision-relevant items here.

## Blocking / must-fix-in-same-PR

- `contact_benchmark`: `tests/benchmark/integration/bm_contact_container.cpp:
  74-87` **compiles the example's header** (`dart::examples::
  contact_benchmark::` world builder + option constants) via `INCLUDE_DIRS
  ${DART_SOURCE_DIR}/examples/contact_benchmark`
  (`tests/benchmark/integration/CMakeLists.txt:67-68`);
  `performance_dashboard_dart6.yml:21` path-triggers on it. Decision: keep
  the example standalone (already in frozen plan) — no relocation needed.

## Update-in-cleanup-PR (low risk)

- `docs/readthedocs/dartpy/user_guide/examples.rst:111-122`: live commands
  for `ssik_analytical_ik` / `ssik_ik_gui` → repoint at demos/py-demos.
- `pixi.toml`: `install-ssik` comment (156-159); dev tasks `ex-atlas-puppet`,
  `ex-atlas-simbicon`, `ex-hello-world`, `py-atlas-puppet`,
  `py-ex-hello-world`, `py-ex-hello-world-gui`; generic `ex`/`py-ex` default
  to hello_world (keep hello_world → default stays valid). Add `demos` /
  `py-demos` tasks.
- `tools/freebsd/patches/patch-examples_wam__ikfast_Helpers.cpp`: FreeBSD
  port patch targets `examples/wam_ikfast/Helpers.cpp` → update path or drop
  patch when wam_ikfast folds into demos.
- `docs/readthedocs/gallery.rst` (+ko .po): atlas_simbicon/tinkertoy links
  pinned to historic commit — stale content only; refresh wording.

## Safe (no external refs)

add_delete_skels, biped_stand, box_stacking, boxes, contact_inverse_dynamics,
cylindrical_constraint, drag_and_drop, dynamic_joint_constraints, empty,
fetch, hardcoded_design, hubo_puppet, human_joint_limits, hybrid_dynamics,
imgui, joint_constraints, mixed_chain, operational_space_control,
point_cloud, rerun, rigid_chain, rigid_cubes, rigid_loop, rigid_shapes,
simple_frames, simulation_event_handler, sleeping, soft_bodies, speed_test,
vehicle, contacts_pointcloud (py).

## Build-system cautions

- MSVC builds examples **by default** (`CMakeLists.txt:527`); Linux/macOS use
  `EXCLUDE_FROM_ALL` (:529). examples/CMakeLists.txt must stay valid on
  Windows at every commit.
- Whole examples/ tree is installed by `CMakeLists.txt:705-708` (source
  install) — removing dirs shrinks that install; fine, but keep README
  accurate.
- CodeQL scans `examples/**` (config lists path; no-op if trimmed).
- Known false positives documented in transcript: bm-boxes/bm-empty
  benchmarks, hello_world.txt test asset, hybrid_dynamics_test.skel,
  "rigid_chain" Skeleton name in tutorials, HeightmapShape class.
