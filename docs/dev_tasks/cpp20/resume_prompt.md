You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase7` (based on `main`)
- Phase 6 PR: https://github.com/dartsim/dart/pull/2373 (merged)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375
- CI/merge are handled by the user.
- Phase 7 changes so far: span inputs for Group/CollisionGroup/Recording,
  CloneableVector, ReferentialSkeleton helpers, Skeleton setAllMemberObjectData,
  and IkFast dof map validation.

What to do next:

1. Finish Phase 7 from `docs/dev_tasks/cpp20/00_plan.md`.
2. Update `docs/dev_tasks/cpp20/01_progress.md` as you go.
3. Open a stacked PR for phase 7.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening a PR.
- Use existing `pixi run ...` entry points only.
