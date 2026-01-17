# Agent Guidelines for DART

This file is a pointer board for agents working in this repository. Keep it concise and expand other documents instead.

## Project Overview

**WHAT**: C++20 physics engine for robotics simulation with Python bindings (dartpy)
**WHY**: Research-grade dynamics for robotics, animation, and machine learning
**HOW**: Build/test with `pixi run` tasks; format with `pixi run lint` before commits

## Quick Commands

```bash
pixi run test-all       # Lint + build + all tests (fails fast)
pixi run lint           # Format code/docs (auto-fixes)
pixi run build          # Build C++ and Python
pixi run test-unit      # Unit tests only (faster)
pixi run test-py        # Python tests only
pixi run check-lint     # Check formatting without fixing
```

Success: "All tests passed!"

If this fails, see `docs/onboarding/ci-cd.md` for troubleshooting.

## Context Loading (IMPORTANT)

**Agents MUST load relevant docs before starting work.** Don't assume context is available.

| Task Type       | Required Reading                                               |
| --------------- | -------------------------------------------------------------- |
| **Any task**    | This file (`AGENTS.md`)                                        |
| Building        | `docs/onboarding/building.md`                                  |
| Testing         | `docs/onboarding/testing.md`                                   |
| Contributing    | `docs/onboarding/contributing.md`, `CONTRIBUTING.md`           |
| Code style      | `docs/onboarding/code-style.md`                                |
| Architecture    | `docs/onboarding/architecture.md`, `docs/onboarding/README.md` |
| CI/CD issues    | `docs/onboarding/ci-cd.md`                                     |
| Python bindings | `docs/onboarding/python-bindings.md`                           |
| Model loading   | `docs/onboarding/io-parsing.md`                                |
| Build system    | `docs/onboarding/build-system.md`                              |

## Slash Commands

Use `/dart-*` commands for consistent, context-aware workflows:

| Command               | Purpose                             |
| --------------------- | ----------------------------------- |
| `/dart-new-task`      | Start new feature/bugfix/refactor   |
| `/dart-resume`        | Continue work from previous session |
| `/dart-fix-issue <#>` | Resolve a GitHub issue              |
| `/dart-fix-ci`        | Debug and fix CI failures           |
| `/dart-review-pr`     | Review or address PR feedback       |
| `/dart-docs-update`   | Update documentation                |

Commands are defined in `.claude/commands/` (works with Claude Code, and similar tools).

## Skills (On-Demand Knowledge)

Skills provide domain-specific knowledge loaded when needed:

| Skill             | When to Load                                   |
| ----------------- | ---------------------------------------------- |
| `dart-build`      | Build system, CMake, dependencies              |
| `dart-test`       | Writing tests, debugging failures              |
| `dart-contribute` | PR workflow, code review, dual-PR for bugfixes |

Skills are in `.claude/skills/` (works with both Claude Code and OpenCode).

## Key Rules

- **Bug fixes**: Require PRs to BOTH `release-6.16` AND `main` branches. See `docs/onboarding/contributing.md`.
- **Commands**: Use `pixi run ...` tasks; don't invent new entry points.
- **Formatting**: Run `pixi run lint` before committing (auto-fixes).
- **Subdirectories**: May have their own `AGENTS.md` for module-specific rules.

## Tool Compatibility

> **Note**: Tool compatibility assumptions below are based on testing as of Jan 2025.
> AI tools evolve rapidly. Verify these assumptions when:
>
> - Setting up a new environment
> - Updating tool versions
> - Experiencing unexpected behavior
>
> See `docs/onboarding/ai-tools.md` for detailed compatibility notes.

| Tool            | Instructions               | Commands             | Skills            |
| --------------- | -------------------------- | -------------------- | ----------------- |
| **Claude Code** | `CLAUDE.md` -> `AGENTS.md` | `.claude/commands/`  | `.claude/skills/` |
| **OpenCode**    | `AGENTS.md`                | `.opencode/command/` | `.claude/skills/` |
| **Gemini CLI**  | `GEMINI.md` -> `AGENTS.md` | Read manually        | Read manually     |
| **Codex**       | `AGENTS.md`                | Read manually        | Read manually     |

## Prompt Templates (Reference)

> [`docs/prompts/`](docs/prompts/AGENTS.md) contains prompt template documentation.
> **Prefer slash commands** over manual prompts when available.

Index: [`docs/prompts/AGENTS.md`](docs/prompts/AGENTS.md)
