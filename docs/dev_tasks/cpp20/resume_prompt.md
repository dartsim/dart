You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase10`
- Phase 9 PR: https://github.com/dartsim/dart/pull/2380
- Phase 8 PR: https://github.com/dartsim/dart/pull/2376 (merged)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375 (merged; empty-span guard fix)
- CI/merge are handled by the user.
- Latest local validation: `DART_PARALLEL_JOBS=1 pixi run test-all` (passed).

What to do next:

1. Push `cpp20/phase10` if needed and open the Phase 10 PR.
2. Update `docs/dev_tasks/cpp20/01_progress.md` and keep PR #2380 current.
3. Start Phase 11 from `docs/dev_tasks/cpp20/00_plan.md` after Phase 10 PR is open.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening a PR.
- Use existing `pixi run ...` entry points only.
