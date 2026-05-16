---
description: retrospect on completed work and capture durable lessons
agent: build
---

Retrospect on completed work and capture durable lessons: $ARGUMENTS

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

## Routing

`docs/ai/components.md` owns routing for durable session learnings. Use this
command as the hindsight workflow; do not duplicate the routing table here.

## Workflow

1. Search existing docs before adding content.
2. Decide whether the lesson is general enough for shared AI infra. Do not
   update AI components after every session.
3. Prefer update, remove, consolidate, or restructure over adding new files.
4. Keep `docs/ai/principles.md` compact; put procedures in the owner docs it
   links to.
5. Avoid ephemeral branch, PR, commit, or username details.
6. If adding a workflow command or skill, edit `.claude/` source files and run
   `pixi run sync-ai-commands`.
7. Run `pixi run lint`; for AI workflow changes, also run
   `pixi run check-ai-commands`.

## Output

- What learning was captured
- Where it was documented
- Checks run
