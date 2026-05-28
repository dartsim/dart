# Branch Cleanup Progress (01)

## Status

- Phases 1, 2 complete; Phase 3 non-`7/*` audit complete with verdicts on all 26
  WIP branches; one MIGRATE-PR (abi-check tooling extraction) carried into a
  follow-up task.
- Code changes: None in cleanup commits; abi-check PR tracked separately.
- Branch deletions: 38 remote branches removed total.

## Completed

### Earlier batch (prior session)

- Read `AGENTS.md` and onboarding build/CI docs.
- Ran `git fetch --all --prune`.
- Audited `origin/7/*` branches against `origin/main` and `origin/release-7.0`.
  All `7/*` branches were ahead of `origin/main` but carried 0-2 unique commits
  vs `origin/release-7.0`.
- Deleted (4): `origin/7/fix_archlinux`, `origin/7/doc`,
  `origin/7/constraint_solver`, `origin/7/soa`.

### Phase 1 closeout batch (2026-05-27)

Re-audited each remaining `7/*` branch against current `origin/main` and verified
that every concept the branch explored is already implemented (typically more
completely) under a tracked plan or durable component. Per-branch verdicts:

| Branch                   | Tip      | Superseded by                                                                  |
| ------------------------ | -------- | ------------------------------------------------------------------------------ |
| `7/v2`                   | Dec 2023 | `dart/simulation/experimental/` (PLAN-050); main never adopted entt            |
| `7/entt`                 | Sep 2023 | Same — entt vendor never adopted                                               |
| `7/coupler_constraint`   | Jun 2023 | `dart/constraint/coupler_constraint.{cpp,hpp}` (bilateral, MimicDofProperties) |
| `7/multicore`            | Jun 2023 | `dart/simulation/experimental/compute/*_executor` (PLAN-030)                   |
| `7/read_from_github`     | May 2023 | `dart/utils/http_resource_retriever.{cpp,hpp}` (libcurl, cached, redirects)    |
| `7/physics_rigid_body`   | Feb 2023 | PLAN-080 rigid-body solver on experimental World                               |
| `7/collision_dev`        | Jan 2023 | Native collision dashboard (PLAN-035/036/037 complete)                         |
| `7/collision_engine`     | Feb 2023 | Same                                                                           |
| `7/ecs_view`             | Feb 2023 | `dart/simulation/experimental/frame/` storage                                  |
| `7/ecs_view_dev`         | Mar 2023 | Same                                                                           |
| `7/lcp_jacobi`           | Jan 2023 | `dart/math/lcp/projection/{jacobi,blocked_jacobi}_solver` + 15 others          |
| `7/buddy_allocator`      | Jan 2023 | `dart/common/pool_allocator.{cpp,hpp}` family                                  |
| `7/object_pool`          | Jan 2023 | Same                                                                           |
| `7/sh2py`                | Feb 2023 | Pixi tasks                                                                     |
| `7/sim_engine`           | Jan 2023 | `dart/simulation/experimental/`                                                |
| `7/multibody`            | Feb 2023 | PLAN-080                                                                       |
| `7/nested_group_product` | Jan 2023 | `dart/math/lie_group/group_product*` (PLAN-100)                                |

Deletion verdict: all 17 superseded — no migration needed.

Deleted (17): `origin/7/buddy_allocator`, `origin/7/collision_dev`,
`origin/7/collision_engine`, `origin/7/coupler_constraint`, `origin/7/ecs_view`,
`origin/7/ecs_view_dev`, `origin/7/entt`, `origin/7/lcp_jacobi`,
`origin/7/multibody`, `origin/7/multicore`, `origin/7/nested_group_product`,
`origin/7/object_pool`, `origin/7/physics_rigid_body`, `origin/7/read_from_github`,
`origin/7/sh2py`, `origin/7/sim_engine`, `origin/7/v2`.

### Phase 2 (2026-05-27)

- Verified `release-7.0` has zero references in `docs/`, `.github/`, `scripts/`,
  `cmake/`, `pixi.toml`, `CMakeLists.txt`, `CHANGELOG.md`, `README.md`,
  `CONTRIBUTING.md`.
- Verified zero remote branches were reachable from `release-7.0` but not from
  `main`.
- Reviewed unique commits: 316 commits ahead of `main`, all 2023-era dependabot
  bumps, an entt vendor commit, a merge from `release-6.13`, and an Issue1445
  fix cherry-picked into files (`dart/dynamics/BodyNode.cpp`,
  `dart/gui/osg/Viewer.{cpp,hpp}`, `dart/gui/osg/detail/CameraModeCallback.cpp`)
  that no longer exist on main (snake_case migration + Filament replaces OSG;
  Issue 1445 itself was closed in 2020).
- Deleted: `origin/release-7.0`.

### Phase 3 non-`7/*` audit batch (2026-05-27)

Audited 40 non-permanent remote branches against current `origin/main` (DART 7)
through PR-state cross-reference and per-branch tip-content investigation.

**Resolved by clear evidence (15 deletions in this batch)**

- Ancient pre-2025 wip (5): `dart8/ecs` (2021-11), `gui_advanced` (2022-08),
  `dev` (2024-03), `v7` (2024-03), `scpeters/mimic_hacking_6.13` (2024-08).
- Closed or already-landed PRs (5): `constraint_solver` (2014 PR + 2024 wip
  reuse with no active PR), `ci/dartpy-version-from-package-xml` (#2327 merged
  to `release-6.16`), `feature/contact_manifold` (#2366 closed; CamelCase paths
  predate snake_case migration), `feature/sim_exp` (#2534 closed; PLAN-050
  complete on main), `hello_world` (#2151 closed).
- Superseded 2025-2026 wip (5):
  - `feature/dartpy7-bindings` — parallel `dart7/` tree abandoned in favor of
    in-place `dart/simulation/experimental/` (PLAN-050).
  - `feature/coll_det` — 135-cmt branch on obsolete `dart/collision/dart/engine/`;
    superseded by PRs #2652/#2688/#2700/#2709 and PLAN-035/036/037.
  - `feature/plane-shape-support` — FCL Halfspace + Bullet GImpact fixes already
    in `tests/dart/test/reference_collision/`.
  - `feature/vsg` — VulkanSceneGraph backend conflicts with documented
    Filament-only decision (PLAN-060/090).
  - `feature/viz-migration` — renderer-agnostic scene API goal achieved via
    Filament; Raylib no longer buildable.
  - `feature/demo_app` — Python demo hub superseded by `demos-app` PLAN-103 work.
  - `workbench-refactor` — OSG-based workbench absorbed into dartsim editor
    PLAN-101.
  - `feature/dart7-gui` — parallel `dart7/` scaffolding approach not taken.
  - `refactor/dart8` — EnTT-on-Composite contradicts roadmap; ECS landed under
    `dart::simulation::experimental` instead.
  - `refactor/enum_names` — PascalCase enum convention completed via PRs
    #2592/#2607; osg backend retired.
  - `feature/ai-agent-optimization` — superseded by current
    `.claude/skills/dart-*` structure (PRs #2649/#2543/#2556/#2557/#2695).
  - `add-claude-github-actions-1767984520572` — stock bot-generated boilerplate;
    never merged.
  - `skels_with_spring_damper` — year-stale OSG example; concept covered by
    built-in joint spring/damper API.
- Misnamed cleanup (1): `nested_ns` — title "Flatten namespaces" misleading;
  actual diff is a `math::suffixes` cleanup on retired PascalCase/osg files.
  Cleanup would be re-derived from scratch against current snake_case main if
  desired.

**Migration PRs queued (4 branches kept as parking lots for now)**

- `task/native-collision-performance-exec` — extract 3 narrow-phase perf
  commits (`sphere_sphere`/`sphere_box`/`capsule_box`/`capsule_sphere`) +
  `scripts/generate_collision_benchmark_manifest.py` as a fresh PR. PLAN-036
  "next step" explicitly invites this slice.
- `refactor/cmake_format` — add `.cmake-format.yaml` / `.gersemirc` /
  `scripts/lint_cmake.py` / `pixi run lint-cmake` task. Re-tune style config
  (branch prefers single-line collapse; main convention favors multi-line)
  before reapplying.
- `build/modernize_cmake_api` — rebase last 2-3 commits (`Refine subsystem
CMake helpers`, `Use dart_add_component to install collision headers`,
  `Add formatting opt-out`) onto main; drop early commits superseded by landed
  helper work.
- `6.16/freebsd_patches` — cherry-pick the single `.gitignore` deduplication
  commit to `release-6.16` (NOT main; main already clean). Branch name is
  misleading; all other 32 commits already merged to `release-6.16`.
- `feature/native-occupancy-grid` — 2nd commit `fe6f236 "Checkpoint native
occupancy grid performance"` is unique perf work post-PR #2686 (DDA ray
  traversal via new `visitRayFreeCells` template, `unordered_set` ->
  `unordered_map`, +151 LOC bench expansion). Keep for rebase + PR.

**Concept-only migrations (5 branches kept for design extraction)**

Branches retained as parking lots while their design intent gets captured as
issues / dev_task / design notes; branches can be deleted after.

- `feature/usd-viewer` — OpenUSD scene loader concept; reopen as issue
  aligned with experimental World shape APIs and Filament/dartsim rendering
  (not OSG).
- `feature/free-joint-energy-benchmarks` — torque-free symplectic integrator
  (midpoint Newton + invariants projection + `check_physics_benchmarks.py`
  evidence-gate pattern); capture as dev_task under
  `docs/dev_tasks/rigid_body_dynamics_solver/`.
- `feature/skel_yaml` — 1991-line strategy doc rejecting #496; compress into a
  PLAN entry on `docs/plans/dashboard.md` or `docs/design/` note + GH issue
  closing #496.
- `refactor/ownership` — `SimulationMode { Design, Simulation }` enum +
  edit-vs-owned skeleton lifecycle design (the `World::createSkeletonFromUri`
  removal half already landed).
- `bm-report` — PR-level benchmark comparison comments alongside the
  Performance Dashboard; new discussion/issue referencing PLAN-080.

**Active (3 branches kept)**

- `demos-app` — in-flight PLAN-102 (frozen) + PLAN-103 (active) work; user
  merges main in regularly (`behind=0`). Lands via its own PR sequence.
- `branch_cleanup` — this dev task; will close once non-`7/*` audit lands and
  the abi-check follow-up resolves.
- `feature/native-occupancy-grid` — see MIGRATE-PR above; also active in the
  sense that the perf checkpoint is recent (2026-05-20).

**Carried into follow-up work (1)**

- `issue/1026_abi_stability` — issue #1026 was closed 2025-12-31 as _deferred_
  ("ABI-stability work is still WIP without a committed timeline ... please
  reopen if a real need emerges or when this work is ready to land"). Verdict:
  extract `scripts/abi_check.py` + `pixi run abi-check` opt-in task (no CI
  gate) as a fresh small PR; delete branch after the PR lands. Tracked as the
  immediate follow-up.

**Active open-PR branches (kept; not part of this audit)**

- `feature/ipc-paper-corpus-manifest` (#2718 OPEN -> main)
- `feature/ipc-scene-boundary-diagnostics` (#2719 OPEN -> ipc-paper-corpus-manifest)
- `feature/ipc-deformable-contact-kernels` (base for PR #2720)
- `feature/ipc-distance-hessian-optimization` (#2720 OPEN -> ipc-deformable-contact-kernels)
- `feature/dartsim-editor-project-browser-layers` (#2716 OPEN -> main)

**Permanent (always kept)**

- `main`, `release-6.16`, `gh-pages`

## In progress

- abi-check tooling extraction PR (origin: verdict on
  `issue/1026_abi_stability`).

## Next steps (Phase 3 closeout)

- Land the abi-check extraction PR (script + pixi task + libabigail dep, no CI
  gate); delete `issue/1026_abi_stability` after PR lands.
- When the user picks up any MIGRATE-PR or MIGRATE-CONCEPT branch, rebase or
  capture as documented above and delete the source branch.
- Remove this task folder per `docs/dev_tasks/README.md` cleanup rule once all
  follow-ups land.
