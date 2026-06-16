---
name: dart-retrospect
description: "DART Retrospect: retrospect on completed work and capture durable lessons"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-retrospect.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-retrospect

Use this skill in Codex to run the DART `dart-retrospect` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-retrospect <arguments>`
- Codex: `$dart-retrospect <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

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
2. Inventory the completed work before routing lessons. Consider domain/design
   choices, public API or package compatibility, CI/review failure modes,
   workflow friction, and cleanup or handoff steps separately so a late
   operational task does not hide feature-level learnings.
3. Decide whether each lesson is general enough for shared AI infra. Do not
   update AI components after every session.
4. Prefer update, remove, consolidate, or restructure over adding new files.
5. Make every durable learning discoverable from its owner surface: update an
   existing owner doc when possible, and when a new durable file is justified,
   link it from the relevant owner index or plan before retiring temporary
   context.
6. Keep `docs/ai/principles.md` compact; put procedures in the owner docs it
   links to.
7. Avoid ephemeral branch, PR, commit, or username details.
8. If adding a workflow command or skill, edit `.claude/` source files and run
   `pixi run sync-ai-commands`.
9. Run `pixi run lint` before committing. For AI docs or adapter changes, also
   run the verification gates from `docs/ai/verification.md`: `pixi run
lint-md`, `pixi run check-lint-md`, `pixi run sync-ai-commands`, `pixi run
check-ai-commands`, `pixi run check-docs-policy`, and `pixi run
check-lint-spell`.

## Output

- What learning was captured
- Where it was documented
- Checks run
