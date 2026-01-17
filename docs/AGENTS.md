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
└── .opencode/command/     ← OpenCode (/dart-*)

Shared Skills (Claude Code + OpenCode)
└── .claude/skills/
    ├── dart-build/SKILL.md
    ├── dart-test/SKILL.md
    └── dart-contribute/SKILL.md

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
- **Running tests?** → `onboarding/testing.md` or load skill `dart-test`
- **Understanding architecture?** → `onboarding/architecture.md`
- **Contributing?** → `onboarding/contributing.md` or load skill `dart-contribute`
- **Adding docs?** → `readthedocs/README.md`
- **AI tool issues?** → `onboarding/ai-tools.md`

See root `AGENTS.md` for repo-wide guidance.
