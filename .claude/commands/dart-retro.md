---
description: run a retrospective and capture durable lessons
argument-hint: "<what-was-completed>"
agent: build
---

Run a retrospective and capture durable lessons: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/AGENTS.md
@docs/ai/principles.md
@docs/ai/components.md
@docs/ai/verification.md
@docs/onboarding/ai-tools.md

## Skip If

Stop without editing when all are true:

- no new patterns, gotchas, or failure modes were discovered
- no workflow improvement is needed
- the learning is too session-specific to help future agents or contributors
- existing docs already cover what happened
- the task was routine and left no durable insight

## Workflow

1. Search existing release-branch docs before adding content.
2. Inventory the completed work before routing lessons. Consider compatibility,
   downstream/Gazebo impact, CI/review failure modes, workflow friction, and
   cleanup or handoff steps separately.
3. Decide whether each lesson is general enough for release-branch AI infra.
   Do not update AI docs after every session.
4. Prefer updating an existing owner doc over adding a new file. Use
   `docs/information-architecture.md` to choose the owner.
5. Keep `docs/ai/principles.md` compact; put procedures in the owner docs it
   links to.
6. Avoid ephemeral branch, PR, commit, or username details.
7. If adding or renaming a workflow command or skill, edit `.claude/` source
   files and run `pixi run sync-ai-commands`.
8. Run `pixi run lint` before committing. For AI workflow changes, also run
   `pixi run sync-ai-commands` and `pixi run check-ai-commands`.

## Output

- What learning was captured
- Where it was documented
- Checks run
