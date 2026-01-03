You are resuming the C++20 modernization task in
`/home/js/dev/dartsim/dart/build_system`.

Current state:

- Branch: `cpp20/phase8`
- PR: pending for phase 8
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375 (empty-span guard fix pushed)
- CI/merge are handled by the user.

What to do next:

1. Continue Phase 8 from `docs/dev_tasks/cpp20/00_plan.md`.
2. Convert parsing helpers (XML/SDF/URDF/Skel) to `std::string_view`.
3. Run `pixi run lint`, then `pixi run test-all`.
4. Open the phase 8 PR and update `docs/dev_tasks/cpp20/01_progress.md`.

Workflow requirements:

- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening a PR.
- Use existing `pixi run ...` entry points only.
