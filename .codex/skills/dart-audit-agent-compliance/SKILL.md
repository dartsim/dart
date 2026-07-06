---
name: dart-audit-agent-compliance
description: "DART Audit Agent Compliance: audit and fix gaps when agents miss or cannot discover documented rules"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-audit-agent-compliance.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-audit-agent-compliance

Use this skill in Codex to run the DART `dart-audit-agent-compliance` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a generated Codex adapter entrypoint.

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
@docs/ai/principles.md
@docs/ai/verification.md
@docs/ai/components.md
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
5. Run the principle audit from `docs/ai/principles.md` and use
   `docs/ai/verification.md` to map audit results to evidence.
6. Run the relevant gate set from `docs/ai/verification.md`. For AI workflow
   changes, that includes generated adapter sync/checks, docs policy, spell,
   Markdown checks, and `pixi run lint`.

## Output

- Root cause for the missed rule
- Files changed and why
- Which audit items were proven by automation vs manual inspection
- Confidence level that the recurrence risk is reduced
