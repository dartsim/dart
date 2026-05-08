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

**Agents MUST load relevant docs before starting work.** Use `@file` syntax to auto-load context:

| Task Type       | Load These Files                                            |
| --------------- | ----------------------------------------------------------- |
| **Any task**    | This file (auto-loaded)                                     |
| Building        | @docs/onboarding/building.md                                |
| Testing         | @docs/onboarding/testing.md                                 |
| Contributing    | @docs/onboarding/contributing.md @CONTRIBUTING.md           |
| Code style      | @docs/onboarding/code-style.md                              |
| Architecture    | @docs/onboarding/architecture.md @docs/onboarding/README.md |
| Theory (math)   | @docs/background/dynamics/ @docs/background/lcp/            |
| CI/CD issues    | @docs/onboarding/ci-cd.md                                   |
| Python bindings | @docs/onboarding/python-bindings.md                         |
| Model loading   | @docs/onboarding/io-parsing.md                              |
| Build system    | @docs/onboarding/build-system.md                            |
| AI tools        | @docs/onboarding/ai-tools.md                                |
| PR reviews      | @docs/onboarding/ai-tools.md (AI review handling rules)     |
| Release work    | @docs/onboarding/release-management.md                      |
| Dev tasks       | @docs/dev_tasks/README.md (when to create, cleanup rules)   |

## Workflow Commands

Use `/dart-*` commands for consistent, context-aware workflows in Claude Code
and OpenCode. In Codex, use the generated equivalent skills as `$dart-*`
(for example, `$dart-fix-ci`).

| Claude/OpenCode                | Codex                          | Purpose                             |
| ------------------------------ | ------------------------------ | ----------------------------------- |
| `/dart-new-task`               | `$dart-new-task`               | Start new feature/bugfix/refactor   |
| `/dart-resume`                 | `$dart-resume`                 | Continue work from previous session |
| `/dart-fix-issue <#>`          | `$dart-fix-issue <#>`          | Resolve a GitHub issue              |
| `/dart-fix-ci`                 | `$dart-fix-ci`                 | Debug and fix CI failures           |
| `/dart-review-pr`              | `$dart-review-pr`              | Review or address PR feedback       |
| `/dart-pr`                     | `$dart-pr`                     | Create a DART pull request          |
| `/dart-manage-pr`              | `$dart-manage-pr`              | Manage CI, review, and cleanup      |
| `/dart-merge-pr`               | `$dart-merge-pr`               | Monitor CI and merge a ready PR     |
| `/dart-docs-update`            | `$dart-docs-update`            | Update documentation                |
| `/dart-improve-docs`           | `$dart-improve-docs`           | Capture durable docs learnings      |
| `/dart-audit-agent-compliance` | `$dart-audit-agent-compliance` | Fix gaps when agents miss rules     |
| `/dart-mechanical-refactor`    | `$dart-mechanical-refactor`    | Behavior-preserving refactor        |
| `/dart-branch-cleanup`         | `$dart-branch-cleanup`         | Analyze or clean stale branches     |
| `/dart-triage-issue`           | `$dart-triage-issue`           | Triage a GitHub issue               |
| `/dart-close-issue`            | `$dart-close-issue`            | Draft/post issue closing message    |
| `/dart-downstream-fix`         | `$dart-downstream-fix`         | Fix downstream-reported DART issue  |
| `/dart-backport-pr`            | `$dart-backport-pr`            | Backport merged `main` PR           |
| `/dart-release-ci-fix`         | `$dart-release-ci-fix`         | Fix release branch CI               |
| `/dart-release-merge-main`     | `$dart-release-merge-main`     | Merge release branch into `main`    |
| `/dart-release-packaging`      | `$dart-release-packaging`      | Prepare release packaging PR        |

Commands are defined in `.claude/commands/` and synced to
`.opencode/command/`. Codex workflow skills are generated from the same command
files into `.codex/skills/`.

## Skills (On-Demand Knowledge)

Skills provide domain-specific knowledge loaded when needed:

| Skill             | When to Load                                    |
| ----------------- | ----------------------------------------------- |
| `dart-build`      | Build system, CMake, dependencies               |
| `dart-ci`         | GitHub Actions and CI troubleshooting           |
| `dart-test`       | Writing tests, debugging failures               |
| `dart-io`         | Model loading and URDF/SDF/MJCF/SKEL parsing    |
| `dart-python`     | dartpy bindings, nanobind, wheels, API patterns |
| `dart-contribute` | PR workflow, code review, dual-PR for bugfixes  |

Skills are in `.claude/skills/` (synced to `.codex/skills/` for Codex).

## Key Rules

- **Bug fixes**: Require PRs to BOTH `release-6.16` AND `main` branches. See `docs/onboarding/contributing.md`.
- **Multi-phase tasks**: Create `docs/dev_tasks/<task>/` for tracking. See `docs/dev_tasks/README.md` for criteria.
- **AI reviews**: NEVER reply to AI-generated review comments (usernames ending in `[bot]` like `chatgpt-codex-connector[bot]`, `github-actions[bot]`, `copilot[bot]`). No inline replies and no acknowledgment comments. Just push the fix silently, then re-trigger with a top-level `@codex review` comment when needed. See `docs/onboarding/ai-tools.md`.
- **Commands**: Use `pixi run ...` tasks; don't invent new entry points.
- **Formatting**: Run `pixi run lint` before committing (auto-fixes).
- **Commit/PR titles**: Do not prefix commit messages or PR titles with agent tags like `[codex]`; use plain descriptive titles.
- **PRs**: Use `.github/PULL_REQUEST_TEMPLATE.md` and set the milestone (`DART 7.0` for `main`, `DART 6.16.x` for `release-6.16`).
- **Subdirectories**: May have their own `AGENTS.md` for module-specific rules.

## Pre-Commit Checklist (MANDATORY)

**STOP before every `git commit`. Verify:**

- [ ] **`pixi run lint`** — ALWAYS run, even for docs-only changes. CI WILL fail without this.
- [ ] `pixi run build` — If C++/Python code changed
- [ ] `pixi run test-unit` — If behavior could be affected
- [ ] **CHANGELOG.md** — Update if adding features, fixing bugs, or making breaking changes
- [ ] **Dev task cleanup** — If task used `docs/dev_tasks/<task>/`, remove folder in this PR (not after merge)

Shortcut: `pixi run test-all` runs lint + build + all tests.

**Why this exists**: Agents often skip `pixi run lint` when focused on the task. CI will catch issues, but fixing post-push wastes time and CI resources. Run lint locally first—EVERY time.

## Tool Compatibility

> **Note**: Codex command/skill sync was verified in May 2026; other tool
> compatibility assumptions below are based on testing as of Jan 2025.
> AI tools evolve rapidly. Verify these assumptions when:
>
> - Setting up a new environment
> - Updating tool versions
> - Experiencing unexpected behavior
>
> See `docs/onboarding/ai-tools.md` for detailed compatibility notes.

| Tool            | Instructions               | Commands                          | Skills                          |
| --------------- | -------------------------- | --------------------------------- | ------------------------------- |
| **Claude Code** | `CLAUDE.md` -> `AGENTS.md` | `.claude/commands/`               | `.claude/skills/`               |
| **OpenCode**    | `AGENTS.md`                | `.opencode/command/`              | `.claude/skills/`               |
| **Codex**       | `AGENTS.md`                | `.codex/skills/`                  | `.codex/skills/`                |
| **Gemini CLI**  | `GEMINI.md` -> `AGENTS.md` | Read `.claude/commands/` manually | Read `.claude/skills/` manually |

<skills_system priority="1">

## Available Skills

<!-- SKILLS_TABLE_START -->
<usage>
When users ask you to perform tasks, check if any of the available skills below can help complete the task more effectively. Skills provide specialized capabilities and domain knowledge.

How to use skills:

- Invoke: `npx openskills read <skill-name>` (run in your shell)
  - For multiple: `npx openskills read skill-one,skill-two`
- The skill content will load with detailed instructions on how to complete the task
- Base directory provided in output for resolving bundled resources (references/, scripts/, assets/)

Usage notes:

- Only use skills listed in <available_skills> below
- Do not invoke a skill that is already loaded in your context
- Each skill invocation is stateless
  </usage>

<available_skills>

<skill>
<name>dart-build</name>
<description>DART Build: CMake, pixi, dependencies, and build troubleshooting</description>
<location>project</location>
</skill>

<skill>
<name>dart-ci</name>
<description>DART CI: GitHub Actions, cache debugging, and platform-specific failures</description>
<location>project</location>
</skill>

<skill>
<name>dart-contribute</name>
<description>DART Contribute: branching, PRs, review workflow, and dual-PR bugfixes</description>
<location>project</location>
</skill>

<skill>
<name>dart-io</name>
<description>DART IO: URDF, SDF, MJCF, SKEL parsers, and dart::io loading</description>
<location>project</location>
</skill>

<skill>
<name>dart-python</name>
<description>DART Python: dartpy bindings, nanobind, wheels, and API patterns</description>
<location>project</location>
</skill>

<skill>
<name>dart-test</name>
<description>DART Test: unit tests, integration tests, CI validation, and debugging</description>
<location>project</location>
</skill>

</available_skills>

<!-- SKILLS_TABLE_END -->

</skills_system>
