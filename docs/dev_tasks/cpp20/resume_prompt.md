You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase11`
- PR (Phase 11): https://github.com/dartsim/dart/pull/2384
- Phase 10 PR: https://github.com/dartsim/dart/pull/2382
- Phase 9 PR: https://github.com/dartsim/dart/pull/2380 (merged)
- Phase 8 PR: https://github.com/dartsim/dart/pull/2376 (merged)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375 (merged; empty-span guard fix)
- CI/merge are handled by the user.
- Latest local validation: `DART_PARALLEL_JOBS=42 pixi run test-all` (passed).

What to do next:

1. Monitor PRs #2382 and #2384 for CI/review feedback.
2. Start the next phase after Phase 11 lands (update `docs/dev_tasks/cpp20/00_plan.md` if a new phase is added).
3. Keep `docs/dev_tasks/cpp20/01_progress.md` current.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening a PR.
- Use existing `pixi run ...` entry points only.
