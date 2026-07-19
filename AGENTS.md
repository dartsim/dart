# Agent Guidelines for DART 6.20

This file is the repository pointer board. Start every task with
`docs/ai/principles.md`, then load only the owner docs needed for the work.

## Project Profile

- **WHAT**: C++17 robotics physics engine with pybind11-based dartpy bindings
- **WHY**: Stable DART 6 LTS compatibility for users and Gazebo/gz-physics
- **HOW**: Use repository `pixi run ...` tasks; run `pixi run lint` before commits

## Quick Commands

```bash
pixi run config       # Configure the default CMake/Ninja build
pixi run build        # Build C++ libraries and utilities
pixi run test         # Build and run C++ tests
pixi run build-py-dev # Build pybind11 dartpy bindings
pixi run test-py      # Run dartpy tests
pixi run test-all     # Build all default CMake targets
pixi run lint         # Format code, docs, TOML, and spelling (auto-fixes)
pixi run check-lint   # Run the non-mutating lint aggregate
```

For downstream-sensitive changes, also run `pixi run -e gazebo test-gz`.
See `docs/onboarding/ci-cd.md` when a gate fails.

## Task-specific Context

| Task                                 | Load these owners                                                                                     |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| Any task                             | `docs/ai/principles.md`                                                                               |
| Architecture or component boundaries | `docs/onboarding/architecture.md`                                                                     |
| Building or dependencies             | `docs/onboarding/building.md`, `docs/onboarding/build-system.md`                                      |
| Testing or simulation evidence       | `docs/onboarding/testing.md`, `docs/ai/verification.md`; use `/dart-verify-sim` or `$dart-verify-sim`  |
| Contribution, branches, or style     | `docs/onboarding/contributing.md`, `docs/onboarding/code-style.md`, `CONTRIBUTING.md`                 |
| Documentation placement              | `docs/README.md`, `docs/information-architecture.md`, `docs/AGENTS.md`                                |
| AI workflows or tooling              | `docs/ai/README.md`, `docs/ai/workflows.md`, `docs/ai/terminology.md`, `docs/onboarding/ai-tools.md`  |
| Planning or autonomous work          | `docs/ai/north-star.md`, `docs/plans/dashboard.md`, `docs/ai/orchestration.md`, `docs/ai/sessions.md` |
| CI failures                          | `docs/onboarding/ci-cd.md`                                                                            |
| Python bindings                      | `docs/onboarding/python-bindings.md`                                                                  |
| Model loading and parsers            | `docs/onboarding/io-parsing.md`                                                                       |
| Release maintenance                  | `docs/onboarding/release-management.md`                                                               |
| Changelog decisions                  | `docs/onboarding/changelog.md`, `docs/onboarding/release-management.md`                               |
| Multi-session dev tasks              | `docs/dev_tasks/README.md`                                                                            |

Subdirectories may provide a closer `AGENTS.md`; instructions accumulate from
the repository root to the working directory.

## DART 6.20 Compatibility Rules

- Branch from `origin/release-6.20` into a non-tracking topic branch; never
  commit directly to `release-*`.
- Preserve C++17, pybind11, `dart::utils` parsers, OSG, installed headers,
  package components, ABI-sensitive interfaces, default simulation behavior,
  and Gazebo/gz-physics compatibility unless a maintainer approves otherwise.
- DART 7 `main` is reference evidence only. Do not import C++23, nanobind,
  `dart::io`, or DART 7-only solver/backend workflows into this branch.
- Bug fixes that apply to DART 6 and DART 7 require separate PRs to the active
  release branch and `main`.
- GitHub mutations, pushes, PR changes, CI reruns, review-thread mutations, and
  branch deletion require explicit maintainer/user approval.
- Never reply to AI-generated review comments; make approved local fixes
  silently.

## AI Capability Sources

Editable workflows and domain skills live in `.claude/commands/` and
`.claude/skills/`. Generated Codex and OpenCode adapters live in
`.agents/skills/` and `.opencode/command/`; do not hand-edit them.

```bash
pixi run sync-ai-commands
pixi run check-ai-commands
pixi run python scripts/setup_ai.py
pixi run python scripts/check_ai_infrastructure.py --doctor
```

Use `$dart-ultrawork` in Codex or `/dart-ultrawork` in Claude/OpenCode for
large, multi-session, or explicitly autonomous work. Its project home is
`docs/dev_tasks/<task>/`.

## Before Every Commit

- Run `pixi run lint`, even for docs-only changes.
- Run `pixi run build` and focused tests when C++ or Python behavior changes.
- Use `dart-changelog` to decide whether `CHANGELOG.md` needs an entry.
- Promote durable facts and remove a completing `docs/dev_tasks/<task>/` folder.
- Run `pixi run install-hooks` once per clone. Its fast staged safety check is
  not a substitute for the full lint requirement; `DART_SKIP_HOOKS=1` is the
  emergency bypass.
