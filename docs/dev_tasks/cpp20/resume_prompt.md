You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase6`
- PR: https://github.com/dartsim/dart/pull/2373
- Merge conflicts with `origin/main` were resolved and pushed.
- CI/merge are handled by the user.

What to do next:

1. Wait for phase 6 to merge.
2. Create `cpp20/phase7` from `origin/main`.
3. Execute Phase 7 from `docs/dev_tasks/cpp20/00_plan.md`.
4. Update `docs/dev_tasks/cpp20/01_progress.md` as you go.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening a PR.
- Use existing `pixi run ...` entry points only.
