# Branch Cleanup Progress (01)

## Status

- Current phase: Phase 1 (branch triage)
- Code changes: None
- Branch deletions: 1 remote branch removed

## Completed

- Read AGENTS.md and onboarding build/CI docs.
- Ran git fetch --all --prune.
- Triaged origin/7/fix_archlinux against origin/main:
  - The unique fix (Issue1445) already exists on origin/main.
  - GUI-specific change is obsolete because dart/gui/osg/Viewer.hpp no longer
    exists on origin/main.
  - Decision: delete remote branch.
  - Deleted: origin/7/fix_archlinux.
- Audited origin/7/* branches vs origin/main and origin/release-7.0.
  - All origin/7/* branches are ahead of origin/main (not merged).
  - Unique commits vs origin/release-7.0 are 0-2 per branch.
- Triaged origin/7/doc against origin/main and origin/release-7.0:
  - No unique commits vs origin/release-7.0.
  - Decision: delete remote branch.
  - Deleted: origin/7/doc.

## Triage results (origin/7/*)

Recommend delete (no unique commits vs origin/release-7.0):
- origin/7/constraint_solver

Small, focused changes (candidate rebase if still desired; otherwise delete):
- origin/7/coupler_constraint (cb1b621: coupled mode for mimic constraint)
- origin/7/read_from_github (c1442e4: GitHub download support for DartResourceRetriever)
- origin/7/object_pool (0599751: ObjectPool utility + tests)
- origin/7/multicore (1da3125: ThreadPool utility)
- origin/7/soa (2f806ee: SoA helper + tests)
- origin/7/nested_group_product (8ee3942: GroupProductMap + tests)
- origin/7/lcp_jacobi (19de46b: LCP helper additions + tests)
- origin/7/ecs_view_dev (6e846c3, a71d165: EntityManager view/hasComponents)
- origin/7/buddy_allocator (031d594: AllocatorBuddy + tests)

Large WIP/experimental changes (plan required if keeping; otherwise delete):
- origin/7/collision_dev (14879d0: collision engine prototype + entt vendoring)
- origin/7/collision_engine (84edcb2: collision engine prototype)
- origin/7/ecs_view (56e6c67: early ECS view prototype)
- origin/7/entt (1d9d0ce: entt vendoring)
- origin/7/multibody (79dd03d: physics multibody prototype)
- origin/7/physics_rigid_body (689cf51: physics rigid body prototype)
- origin/7/sh2py (7b1cca7: CI/build script prototype)
- origin/7/sim_engine (d7c2649: simulation engine prototype)
- origin/7/v2 (e8dbf1e, ef901be: v2 component prototype)

## In progress

- Confirm which origin/7/* branches should be deleted vs revived.

## Next steps

- Delete approved origin/7/* branches.
- If any small branches are kept, rebase onto origin/main in a cleanup branch
  and open PRs.
- If any large WIP branches are kept, create a scoped plan under
  docs/dev_tasks/branch_cleanup/ and execute in phases.
- Prepare for removal of origin/release-7.0 once dependents are resolved.
- Track deletions, PRs, and open questions per branch.

## Backlog (for later)

- Audit non-7/* remote branches (feature/*, refactor/*, dart8/*, etc) for
  cleanup.
- Remove origin/release-7.0 after dependent branches are resolved.
