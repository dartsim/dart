---
description: audit and fix gaps when agents miss documented rules
agent: build
---

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
