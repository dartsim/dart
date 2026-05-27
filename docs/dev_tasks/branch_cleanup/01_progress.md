# Branch Cleanup Progress (01)

## Status

- Current phase: Phase 2 (`release-7.0` removal) complete; Phase 3 (closeout) and
  backlog (non-`7/*` audit) pending.
- Code changes: None.
- Branch deletions: 23 remote branches removed total.

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

## In progress

- None.

## Next steps (Phase 3 closeout)

- Update `00_plan.md` to mark all phases done.
- Promote any durable findings, then remove this task folder per
  `docs/dev_tasks/README.md` cleanup rule.
- Decide whether the non-`7/*` audit (see Backlog) merits a fresh dev task or a
  one-shot triage in a single PR.

## Backlog (non-`7/*` audit)

Remaining non-permanent remote branches at the time of writing fall into these
buckets:

- **Ancient stale wip (pre-2025)**: `dart8/ecs` (2021-11), `gui_advanced`
  (2022-08), `dev` (2024-03), `v7` (2024-06), `scpeters/mimic_hacking_6.13`
  (2024-08).
- **Mid-2025 to early-2026 `feature/*`, `refactor/*`, `task/*`, `ci/*`,
  `build/*`, `issue/*` branches**: ~28 branches; need per-branch verification
  of merged-PR / superseded / active state.
- **Active PR branches (keep)**: `feature/ipc-paper-corpus-manifest` (#2718),
  `feature/ipc-scene-boundary-diagnostics` (#2719),
  `feature/dartsim-editor-project-browser-layers` (#2716).
- **Permanent**: `main`, `release-6.16`, `gh-pages`.
