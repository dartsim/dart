You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase9` (based on `main`)
- Phase 9 PR: pending (`pixi run lint`, `pixi run test-all` passed)
- Phase 8 PR: https://github.com/dartsim/dart/pull/2376
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375
- CI/merge are handled by the user.

What to do next:

1. Push `cpp20/phase9` to origin.
2. Open the phase 9 PR and update `docs/dev_tasks/cpp20/01_progress.md`.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening a PR.
- Use existing `pixi run ...` entry points only.
