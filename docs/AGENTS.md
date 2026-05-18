# docs/

Agent entry point for DART documentation.

## AI Documentation Architecture

```
DART AI Documentation Structure
===============================

Root Entry Points
├── AGENTS.md          ← Concise pointer board and session context table
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
    ├── ai/                ← AI-native principles, mission, and workflow policy
    ├── assets/            ← Source-controlled docs assets outside RTD static
    ├── background/        ← Theory and research foundations
    ├── design/            ← Durable technical design proposals
    ├── dev_tasks/         ← Active multi-session task tracking
    ├── doxygen/           ← C++ API Doxygen inputs
    ├── plans/             ← Living roadmap, priority, open gaps, and gates
    ├── onboarding/        ← Developer knowledge base and onboarding
    │   └── ai-tools.md    ← Tool compatibility notes
    ├── python_api/        ← Reusable dartpy API module pages
    └── readthedocs/       ← User documentation
```

## Quick Navigation

| Directory                          | Purpose                                                 |
| ---------------------------------- | ------------------------------------------------------- |
| `ai/`                              | AI-native mission, workflow policy, and verification    |
| `assets/`                          | Shared docs assets outside RTD `_static/`               |
| `background/`                      | Theory/research foundations                             |
| `design/`                          | Durable technical design proposals                      |
| `dev_tasks/`                       | Active development task tracking                        |
| `doxygen/`                         | C++ API Doxygen inputs                                  |
| `onboarding/`                      | Developer knowledge base (architecture, build, testing) |
| `onboarding/ai-tools.md`           | AI tool compatibility notes                             |
| `onboarding/api-boundaries.md`     | Public vs internal API policy                           |
| `onboarding/release-management.md` | Release workflow guidance                               |
| `plans/`                           | Living roadmap, priority, gaps, gates, and lifecycle    |
| `python_api/`                      | Reusable dartpy API module pages                        |
| `readthedocs/`                     | Published user documentation (Sphinx/RST)               |

## By Task

- **Building DART?** → `onboarding/building.md` or load skill `dart-build`
- **Debugging CI?** → `onboarding/ci-cd.md` or load skill `dart-ci`
- **Running tests?** → `onboarding/testing.md` or load skill `dart-test`
- **Loading models?** → `onboarding/io-parsing.md` or load skill `dart-io`
- **Working on dartpy?** → `onboarding/python-bindings.md` or load skill `dart-python`
- **Understanding architecture?** → `onboarding/architecture.md`
- **Writing durable design rationale?** → `design/README.md` +
  `onboarding/api-boundaries.md`
- **Understanding theory?** → `background/README.md`
- **Designing public APIs?** → `onboarding/api-boundaries.md`
- **Maintaining API docs?** → `onboarding/api-documentation.md`
- **Revising project plans?** → `plans/README.md` or use `/dart-plan-update`
- **Choosing next AI work?** → `ai/README.md#choosing-the-next-task` or use
  `/dart-next` / `$dart-next`
- **Contributing?** → `onboarding/contributing.md` or load skill `dart-contribute`
- **Release workflow?** → `onboarding/release-management.md` or use `/dart-release-*`
- **Adding docs?** → `readthedocs/README.md`
- **AI tool issues?** → `onboarding/ai-tools.md`

See root `AGENTS.md` for repo-wide guidance.
Load `docs/ai/principles.md` for compact AI-infra axioms; it is the source of
truth for principles, while `AGENTS.md` stays a pointer board.
