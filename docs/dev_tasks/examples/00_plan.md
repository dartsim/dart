# Examples Consolidation Plan (00)

## Status

- In progress: sanity-check example discovery and runner tasks after removals.
- Next: confirm Python example alignment with the updated C++ catalog.
- Completed: removed `rerun` placeholder example (no sources).
- Completed: removed `speed_test`; keep `perf_headless_simulation` as the
  headless
  performance-focused example and rely on `tests/benchmark` for benchmarking.

## Goals

- Reduce redundant or inconsistent examples for onboarding.
- Keep example coverage aligned with supported features and performance scope.
- Merge the Python hello world variants into one example with a GUI flag.

## Constraints

- Every retained example implies a supported, maintained feature with strong
  performance expectations.
- Remove placeholders and experimental examples rather than keeping them.
- Avoid adding more per-example README files; update root indexes instead.

## Phases

1. Decide keep/drop/merge list and record rationale.
2. Apply C++ example removals/merges; update `examples/CMakeLists.txt` and
   `examples/README.md`.
3. Merge Python hello world examples; update `pixi.toml` and
   `python/examples/README.md`.
4. Sanity-check example discovery and runner tasks.

## Decision Log

- Removed `rerun` (placeholder, no sources).
- Removed `speed_test`; keep `perf_headless_simulation` as the headless example
  and
  rely on `tests/benchmark/` for performance coverage.
- Confirmed `speed_test` coverage via
  `tests/benchmark/dynamics/bm_kinematics.cpp` (BM_Kinematics, BM_Dynamics).
- Renamed `atlas_puppet`, `g1_puppet`, and `hubo_puppet` to `ik_atlas`,
  `ik_g1`, and `ik_hubo`.
- Consolidate humanoid IK into a single `ik_humanoid` example with modular
  robot implementations for Atlas, G1, and Hubo.
- Removed `ik_atlas`, `ik_g1`, and `ik_hubo` example directories after
  consolidation into `ik_humanoid`.
- Moved humanoid robot implementations under `examples/ik_humanoid/robots/`.
- Renamed `atlas_simbicon` to `control_walking_humanoid` to match the
  feature-first naming pattern.
- Renamed `biped_stand` -> `control_balance_biped`,
  `operational_space_control` -> `control_operational_space`,
  `vehicle` -> `control_vehicle`, and `wam_ikfast` -> `ik_analytic_wam`.
- Removed per-example C++ pixi tasks; use `pixi run ex -- --list` to discover
  targets and `pixi run ex -- <example>` to run them.
- Moved `simple_frames` under Rigid Bodies and Frames in the examples index and
  fixed the example render path by attaching visual aspects to the frames.
- Adopted example naming prefixes by category (e.g., `control_*`, `ik_*`) and
  will apply the pattern across the remaining sections.
- Renamed joint/constraint examples to the `joint_*` prefix and updated the
  matching Python examples (`joint_chain`, `joint_loop`).
- Renamed collision examples to the `collision_*` prefix.
- Renamed `fetch` -> `model_fetch` and `unified_loading` -> `io_unified_loading`.
- Renamed `mixed_chain` -> `hybrid_mixed_chain`.
- Renamed visualization and interaction examples to the `viz_*` prefix.
- Renamed Python visualization examples to `viz_drag_and_drop` and `viz_imgui`.
- Renamed `headless_simulation` -> `perf_headless_simulation`.
- Renamed `csv_logger` -> `tool_csv_logger` and `point_cloud` -> `viz_point_cloud`.
- Merged Python `hello_world` and `hello_world_gui` into one example with a
  `--gui` flag.
- Renamed Python `atlas_puppet` -> `ik_atlas`.
- Moved `hybrid_dynamics` to Control and renamed it to `control_actuator_modes`.
- Moved `joint_lcp_solvers` to Tools and renamed it to `tool_lcp_solvers`.
- Removed `rigid_boxes`, `joint_coupler`, `joint_constraints`, and
  `viz_simulation_event_handler`.
- Removed `viz_raylib`.
- Removed `joint_free_cases` and `joint_human_limits` (advanced validation and
  custom constraint demos).
- Removed `tool_lcp_solvers`, `viz_tinkertoy`, and `viz_point_cloud` (large UI
  or external dependency scope).
- Kept `control_walking_humanoid` and `ik_analytic_wam` as supported advanced
  examples (documented, user-facing features).
- Reviewed example names and category placement against README goals; no further
  renames required.

## Naming Prefix Plan

### 00 Getting Started

- `hello_world` (no change)

### 01 Rigid Bodies and Frames

- `simple_frames` -> `frame_hierarchy`
- `hardcoded_design` -> `rigid_hardcoded_design`
- `rigid_cubes` -> `rigid_cubes` (no change)

### 02 Joints and Constraints

- `mimic_pendulums` -> `joint_mimic_pendulums`
- `rigid_chain` -> `joint_chain`
- `rigid_loop` -> `joint_loop`

### 03 Collisions and Contacts

- `box_stacking` -> `collision_box_stacking`
- `capsule_ground_contact` -> `collision_capsule_ground_contact`
- `heightmap` -> `collision_heightmap`
- `rigid_shapes` -> `collision_rigid_shapes`

### 04 Control and IK

- `control_balance_biped` (no change)
- `hybrid_dynamics` -> `control_actuator_modes`
- `control_operational_space` (no change)
- `control_vehicle` (no change)
- `control_walking_humanoid` (no change)
- `ik_analytic_wam` (no change)
- `ik_humanoid` (no change)

### 05 IO and Models

- `fetch` -> `model_fetch`
- `unified_loading` -> `io_unified_loading`

### 06 Soft and Hybrid

- `mixed_chain` -> `hybrid_mixed_chain`
- `soft_bodies` (no change)

### 07 Visualization and Interaction

- `add_delete_skels` -> `viz_add_delete_skels`
- `drag_and_drop` -> `viz_drag_and_drop`
- `empty` -> `viz_empty`
- `imgui` -> `viz_imgui`
- `polyhedron_visual` -> `viz_polyhedron_visual`

### 08 Performance and Scaling

- `headless_simulation` -> `perf_headless_simulation`

### 09 Integration and Tools

- `csv_logger` -> `tool_csv_logger`

## Current Suggestions (Pending Decisions)

### Strong Removals (redundant/experimental/placeholder)

None (all strong removal candidates applied).

### Likely Removals (tighten supported scope)

None (remaining examples are in-scope).
