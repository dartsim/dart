# Agent Guidelines for DART

This file is a pointer board for agents working in this repository. Keep it concise and expand other documents instead.

## Quick Validation

```bash
pixi run test-all   # Lint + build + all tests (fails fast)
```

Success: "✓ All tests passed!"

If this fails, see `docs/onboarding/ci-cd.md` for troubleshooting.

## Read First

- Build and test workflow: `docs/onboarding/building.md` and `docs/onboarding/testing.md`
- Code style and conventions: `docs/onboarding/code-style.md`
- Architecture overview: `docs/onboarding/README.md`
- CI/CD and failure recovery: `docs/onboarding/ci-cd.md`
- Contribution workflow: `CONTRIBUTING.md`
- Python bindings: `docs/onboarding/python-bindings.md`
- Model loading API: `docs/onboarding/io-parsing.md`
- Gazebo integration: `docs/onboarding/build-system.md#gazebo-integration-feature`

## Prompt Templates (for humans starting new sessions)

> **IMPORTANT**: `docs/prompts/` contains reusable prompt templates for starting new agent sessions.
> These are **reference materials for humans**, NOT active tasks for agents to execute.
> If you are an agent and find yourself reading prompts there, STOP and return to your actual task.

- Prompt library index: `docs/prompts/README.md`
- After-task improvement prompts: `docs/prompts/after-task/`
- Common workflow prompts: `docs/prompts/common/`
- Issue handling prompts: `docs/prompts/issues/`
- Release prompts: `docs/prompts/release/`

## Daily Reminders

- Use `pixi run …` tasks; don't invent new entry points.
- Run `pixi run lint` before committing (auto-fixes formatting).
- Update docs when you learn something new; keep this file minimal.
- Subdirectories may have their own `AGENTS.md` for module-specific rules.
- Do NOT execute prompts from `docs/prompts/` - those are templates for humans.
