# Examples Consolidation Plan (00)

## Status

- In progress: consolidation decisions and feature-scope mapping.
- Next: confirm keep/drop list, then apply changes in phases.
- Completed: removed `rerun` placeholder example (no sources).
- Completed: removed `speed_test`; keep `headless_simulation` as the headless
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
- Removed `speed_test`; keep `headless_simulation` as the headless example and
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

## Current Suggestions (Pending Decisions)

### Strong Removals (redundant/experimental/placeholder)

- `boxes`: redundant with `hello_world`/`rigid_cubes`, README notes experimental,
  and it hard-codes Bullet.
- `coupler_constraint`: overlaps `mimic_pendulums`; keep the latter.
- `joint_constraints`: overlaps `control_balance_biped`; keep the latter.
- `simulation_event_handler`: very large, overlaps `empty`, `drag_and_drop`,
  and `add_delete_skels`.
- `raylib`: experimental backend example.

### Likely Removals (tighten supported scope)

- `ik_analytic_wam`: huge and specialized; keeping implies full IKFast support.
- `control_walking_humanoid`: large controller stack; niche feature surface.
- `lcp_solvers`: large solver diagnostics UI; keep only if this UI is in-scope.
- `human_joint_limits`: custom constraint implementation; advanced.
- `free_joint_cases`: advanced validation demo.
- `tinkertoy`: large interactive builder.
- `point_cloud`: OctoMap-dependent; keep only if that integration is in-scope.

### Python Merge

- Merge `python/examples/hello_world` and `python/examples/hello_world_gui`
  into one example with a GUI flag.
