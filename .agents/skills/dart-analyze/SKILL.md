---
name: dart-analyze
description: "DART Analyze: analyze repository evidence without editing"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-analyze.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-analyze

Use this skill in Codex to run the DART `dart-analyze` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-analyze <arguments>`
- Codex: `$dart-analyze <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Analyze repository evidence without editing: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/workflows.md
@docs/ai/verification.md
@docs/onboarding/ai-tools.md

## Workflow

1. Restate the question and the read-only boundary. Do not edit files, stage
   changes, or perform GitHub, CI, branch, or review-thread mutations.
2. Inspect local state enough to avoid misreading user work:
   ```bash
   git status --short --branch
   git diff --stat
   ```
3. Build an evidence set from repository files, tests, docs, generated
   artifacts, and command output. Load task-specific docs from `AGENTS.md` when
   the question names a subsystem.
4. Separate direct evidence from inference and unknowns. Do not present an
   inference as a fact, and do not turn a read-only answer into an
   implementation plan.
5. Rank explanations, risks, or options by confidence when multiple readings
   are plausible. Use concrete file and line references for material claims.
6. If current external documentation or standards are needed for correctness,
   gather source-backed evidence and label date/version context explicitly.
7. Stop when the synthesis answers the question with enough evidence, or report
   the exact proof source that is unavailable.

## Output

Report:

- the question answered and the scope inspected;
- ranked synthesis with confidence;
- evidence, inference, and unknowns as separate sections;
- any discriminating read-only probe that would reduce remaining uncertainty;
- confirmation that no local edits or external mutations were performed.
