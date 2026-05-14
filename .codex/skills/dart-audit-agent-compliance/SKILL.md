---
name: dart-audit-agent-compliance
description: "DART Audit Agent Compliance: audit and fix gaps when agents miss documented rules"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-audit-agent-compliance.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-audit-agent-compliance

Use this skill in Codex to run the DART `dart-audit-agent-compliance` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-audit-agent-compliance <arguments>`
- Codex: `$dart-audit-agent-compliance <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Audit agent compliance issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ai-tools.md
@docs/onboarding/contributing.md

## Incident Inputs

- Rule violated: $1
- Where documented: $2
- Actual behavior: $3
- Expected behavior: $4

Treat missing positional values as details to infer from the user request or ask about before editing.

## Workflow

1. Locate the exact existing rule and confirm it is still correct.
2. Diagnose why it was missed:
   - buried in prose
   - wrong file for the task type
   - weak emphasis
   - duplicated or conflicting guidance
   - not referenced from relevant commands or skills
3. Prefer restructuring existing docs over adding duplicate content.
4. Improve visibility with one or more focused changes:
   - move the rule to a loaded file
   - make it scannable with a checklist or mandatory marker
   - consolidate duplicate guidance
   - add cross-references from relevant commands or skills
5. Run `pixi run lint` and, for AI workflow changes, `pixi run check-ai-commands`.

## Output

- Root cause for the missed rule
- Files changed and why
- Confidence level that the recurrence risk is reduced
