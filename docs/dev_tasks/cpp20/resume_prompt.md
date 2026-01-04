You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase14`
- PR (Phase 14): https://github.com/dartsim/dart/pull/2390
- Phase 13 PR: https://github.com/dartsim/dart/pull/2387
- Phase 12 PR: https://github.com/dartsim/dart/pull/2385
- Phase 11 PR: https://github.com/dartsim/dart/pull/2384
- Phase 10 PR: https://github.com/dartsim/dart/pull/2382 (merged)
- Phase 9 PR: https://github.com/dartsim/dart/pull/2380 (merged)
- Phase 8 PR: https://github.com/dartsim/dart/pull/2376 (merged)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375 (merged; empty-span guard fix)
- CI/merge are handled by the user.
- Latest local validation: `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run test-all`;
  `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run -e gazebo test-gz`
  (passed).

What to do next:

1. Monitor PRs #2384, #2385, #2387, and #2390 for CI/review feedback.
2. Start the next phase after Phase 14 lands (update `docs/dev_tasks/cpp20/00_plan.md` if a new phase is added).
3. Keep `docs/dev_tasks/cpp20/01_progress.md` current.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` and `pixi run -e gazebo test-gz` before opening a PR (do not modify Gazebo sources).
- Use `DART_PARALLEL_JOBS` set to 2/3 of `nproc` for builds/tests.
- Use existing `pixi run ...` entry points only.
