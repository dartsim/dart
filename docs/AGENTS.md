# docs/

Agent entry point for DART documentation.

## AI Documentation Architecture

```
DART AI Documentation Structure
===============================

Entry Points (all redirect to AGENTS.md)
├── AGENTS.md          ← Single source of truth
├── CLAUDE.md          ← Redirect for Claude Code
└── GEMINI.md          ← Redirect for Gemini CLI

Tool-Specific Commands
├── .claude/commands/      ← Claude Code (/dart-*)
├── .opencode/command/     ← OpenCode (/dart-*)
└── .codex/skills/         ← Codex ($dart-* generated workflow skills)

Shared Skills
├── .claude/skills/        ← Claude Code + OpenCode
└── .codex/skills/         ← Codex (synced)
    ├── dart-build/SKILL.md
    ├── dart-ci/SKILL.md
    ├── dart-contribute/SKILL.md
    ├── dart-io/SKILL.md
    ├── dart-python/SKILL.md
    └── dart-test/SKILL.md

Knowledge Base
└── docs/
    ├── onboarding/        ← Developer guides
    │   └── ai-tools.md    ← Tool compatibility notes
    ├── prompts/           ← Template reference
    └── readthedocs/       ← User documentation
```

## Quick Navigation

| Directory                     | Purpose                                            |
| ----------------------------- | -------------------------------------------------- |
| `onboarding/`                 | Developer guides (architecture, building, testing) |
| `onboarding/ai-tools.md`      | AI tool compatibility notes                        |
| [prompts/](prompts/AGENTS.md) | Prompt templates (prefer slash commands)           |
| `dev_tasks/`                  | Active development task tracking                   |
| `readthedocs/`                | Published user documentation (Sphinx/RST)          |

## By Task

- **Building DART?** → `onboarding/building.md` or load skill `dart-build`
- **Debugging CI?** → `onboarding/ci-cd.md` or load skill `dart-ci`
- **Running tests?** → `onboarding/testing.md` or load skill `dart-test`
- **Loading models?** → `onboarding/io-parsing.md` or load skill `dart-io`
- **Working on dartpy?** → `onboarding/python-bindings.md` or load skill `dart-python`
- **Understanding architecture?** → `onboarding/architecture.md`
- **Contributing?** → `onboarding/contributing.md` or load skill `dart-contribute`
- **Adding docs?** → `readthedocs/README.md`
- **AI tool issues?** → `onboarding/ai-tools.md`

See root `AGENTS.md` for repo-wide guidance.
