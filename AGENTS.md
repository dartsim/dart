# Agent Guidelines for DART

This file is a pointer board for agents working in this repository. Keep it
concise and expand other documents instead.

## Project Overview

**WHAT**: C++23 physics engine for robotics simulation with Python bindings (dartpy)
**WHY**: Research-grade dynamics for robotics, animation, and machine learning
**HOW**: Build/test with `pixi run` tasks; format with `pixi run lint` before commits

## Quick Commands

```bash
pixi run test-all       # Lint + build + all tests (fails fast)
pixi run lint           # Format code/docs (auto-fixes)
pixi run build          # Build C++ and Python
pixi run test-unit      # Unit tests only (faster)
pixi run test-py        # Python tests only
pixi run -e cuda test-all # CUDA full validation on Linux CUDA hosts
pixi run check-lint     # Check formatting without fixing
```

Success: "All tests passed!"

If this fails, see `docs/onboarding/ci-cd.md` for troubleshooting.

## Context Loading (IMPORTANT)

**Agents MUST load relevant docs before starting work.** Start every session by
reading `docs/ai/principles.md`, then load the task-specific docs below. Tools
that support `@file` references may use them, but the paths below are the
portable source of truth.

| Task Type                             | Load These Files                                                                                                                                                   |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Any task**                          | This file (auto-loaded), `docs/ai/principles.md`                                                                                                                   |
| Project direction                     | `docs/ai/north-star.md`                                                                                                                                            |
| Building                              | `docs/onboarding/building.md`                                                                                                                                      |
| Testing                               | `docs/onboarding/testing.md`                                                                                                                                       |
| Contributing                          | `docs/onboarding/contributing.md`, `CONTRIBUTING.md`                                                                                                               |
| Code style                            | `docs/onboarding/code-style.md`                                                                                                                                    |
| Architecture                          | `docs/onboarding/architecture.md`, `docs/onboarding/README.md`                                                                                                     |
| DART 7 architecture vision            | `docs/readthedocs/architecture.md` (multi-physics/solver/backend one-paper), `docs/design/simulation_solver_architecture.md`                                       |
| Architecture hardening / work packets | `docs/design/dart7_architecture_assessment.md` (verified findings + standing rule), `docs/plans/solver-family-intake.md`, `docs/ai/orchestration.md`               |
| Public API work                       | `docs/onboarding/api-boundaries.md`                                                                                                                                |
| Theory/research foundations           | `docs/background/README.md`                                                                                                                                        |
| CI/CD issues                          | `docs/onboarding/ci-cd.md`                                                                                                                                         |
| Python bindings                       | `docs/onboarding/python-bindings.md`                                                                                                                               |
| Model loading                         | `docs/onboarding/io-parsing.md`                                                                                                                                    |
| Build system                          | `docs/onboarding/build-system.md`                                                                                                                                  |
| Profiling / performance               | `docs/onboarding/profiling.md`                                                                                                                                     |
| AI tools                              | `docs/ai/README.md`, `docs/ai/principles.md`, `docs/ai/north-star.md`, `docs/onboarding/ai-tools.md`                                                               |
| Planning                              | `docs/ai/principles.md`, `docs/ai/north-star.md`, `docs/plans/README.md`, `docs/plans/dashboard.md`, `docs/plans/north-star-roadmap.md`, `docs/ai/verification.md` |
| PR reviews                            | `docs/onboarding/ai-tools.md` (AI review handling rules)                                                                                                           |
| Release work                          | `docs/onboarding/release-management.md`                                                                                                                            |
| Changelog work                        | `docs/onboarding/changelog.md`, `docs/onboarding/release-roadmap.md`, `docs/onboarding/release-management.md`                                                      |
| Dev tasks                             | `docs/dev_tasks/README.md` (when to create, cleanup rules)                                                                                                         |

## AI Workflows And Skills

Use `/dart-*` commands in Claude Code/OpenCode and generated `$dart-*` skills in
Codex. `docs/ai/workflows.md` owns the workflow and skill catalog.
Use `/dart-next` or `$dart-next` when an agent should select the next bounded
task from the north star, plan dashboard, dev-task state, issues, PRs, or CI;
pass `focus=<topic>` to prefer an area without hard-coding the outcome.

Editable sources live in `.claude/commands/` and `.claude/skills/`. Generated
OpenCode and Codex entrypoints live in `.opencode/command/` and
`.codex/skills/`; do not hand-edit generated files.

## Key Rules

- **Bug fixes**: Require PRs to BOTH the active DART 6 LTS branch (highest maintained `release-6.*`, currently `release-6.20`) AND `main`. See `docs/onboarding/contributing.md`.
- **Multi-phase tasks**: Create `docs/dev_tasks/<task>/` for tracking. Promote durable artifacts before completion and delete the task folder in the completing PR. Relocate any deferred or hard-blocked remaining work to a durable home (design/plan/dashboard) and ask the human before retiring if it can't be completed. See `docs/dev_tasks/README.md` for criteria and cleanup rules.
- **AI reviews**: NEVER reply to AI-generated review comments (usernames ending in `[bot]` like `chatgpt-codex-connector[bot]`, `github-code-quality[bot]`, `github-actions[bot]`, `copilot[bot]`). No inline replies and no acknowledgment comments. Make local fixes silently. Pushes, PR comments, thread resolution, review re-triggers, and other GitHub mutations require explicit maintainer/user approval. See `docs/onboarding/ai-tools.md`.
- **Commands**: Use `pixi run ...` tasks; don't invent new entry points.
- **Formatting**: Run `pixi run lint` before committing (auto-fixes).
- **Commit/PR titles**: Do not prefix commit messages or PR titles with agent tags like `[codex]`; use plain descriptive titles.
- **PRs**: Use `.github/PULL_REQUEST_TEMPLATE.md` and set the milestone (`DART 7.0` for `main`, branch-matching DART 6.x release milestone for the active DART 6 LTS branch).
- **PR pushes**: Before every push to a PR branch, first merge the latest base branch (usually `main`) into it — merge, never rebase a published PR branch. The local merge is routine; the push still needs approval. See `docs/onboarding/ai-tools.md`.
- **Subdirectories**: May have their own `AGENTS.md` for module-specific rules.

## Pre-Commit Checklist (MANDATORY)

**STOP before every `git commit`. Verify:**

- [ ] **`pixi run lint`** — ALWAYS run, even for docs-only changes. CI WILL fail without this.
- [ ] `pixi run build` — If C++/Python code changed
- [ ] `pixi run test-unit` — If behavior could be affected
- [ ] **CHANGELOG.md** — Update according to `docs/onboarding/changelog.md` if adding features, fixing bugs, or making breaking changes
- [ ] **Dev task cleanup** — If task used `docs/dev_tasks/<task>/`, promote durable artifacts and remove the folder in this PR (not after merge)

Shortcut: `pixi run test-all` runs lint + build + all tests.
On Linux hosts with a visible NVIDIA CUDA runtime, also run
`pixi run -e cuda test-all`; it preserves the CUDA Pixi environment and runs the
CUDA runtime smoke path automatically when a CUDA device is detected.

**Why this exists**: Agents often skip `pixi run lint` when focused on the task. CI will catch issues, but fixing post-push wastes time and CI resources. Run lint locally first—EVERY time.

## Tool Compatibility

See `docs/onboarding/ai-tools.md` for Claude Code, Codex, Gemini CLI, and
OpenCode compatibility details. Keep this root file as a pointer board, not a
second tool registry.
