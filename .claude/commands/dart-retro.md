---
description: improve the AI harness from successful or unsatisfactory task outcomes
argument-hint: "[task, PR, or outcome; defaults to this session]"
agent: build
---

Improve future task execution through a retrospective: $ARGUMENTS

Use the original request and session evidence to improve the harness for future
similar tasks: better quality, completeness, or efficiency. Successful,
unsatisfactory, incomplete, and blocked outcomes qualify. PR merge and dev-task
retirement do not alone establish that the user's goal was met.

## Required Reading

@AGENTS.md
@docs/AGENTS.md
@docs/ai/principles.md
@docs/ai/components.md
@docs/ai/verification.md

Load task owners as needed; use `docs/onboarding/ai-tools.md` for tool/runtime
questions. `docs/ai/components.md` owns placement of accepted learnings; this
workflow owns the retrospective method.

## Skip If

Conclude without harness edits when the evidence supports no reusable
improvement: existing guidance already covers the lesson and is discoverable,
the finding is task-specific, or no change has a supported benefit. State why
and report missing evidence; success, failure, or a merged PR alone is not a
skip condition.

## Workflow

1. **Reconstruct intent and outcome.** Default to the current session. Compare
   the initial request and later scope decisions with artifacts and results.
   Inspect relevant history around decisions, corrections, and failures,
   including substantive domain work before CI/review/closeout. State missing
   evidence; do not load every log by default.
2. **Find the harness contribution.** Connect useful decisions, rework, missed
   requirements, and wasted context/tool cycles to instructions, routing, tools,
   or gates. Separate observations from inferred causes, implementation bugs,
   and external blockers. Check existing owners and executable coverage before
   proposing rules; investigate why existing guidance was missed.
3. **Choose a testable improvement.** State the observed decision, causal gap,
   owner, and what an agent starting from the same brief should do differently.
   Name the expected benefit, a check that could disprove it, and successful
   constraints to preserve.
4. **Improve existing owners.** Prefer removal, consolidation, or rewriting to
   appending rules or files; repair discovery when guidance already exists.
   Follow `docs/ai/components.md` for placement and adapter regeneration. Keep
   session identifiers out of durable guidance. Preserve model/effort/action
   limits; a retrospective does not reopen the original implementation or
   authorize new GitHub mutations.
5. **Validate proportionately.** For consequential instruction changes, replay
   the observed decision and a contrasting similar task; use fresh controlled
   agents when permitted and useful. Distinguish structural checks, predicted
   benefits, and measured outcomes. Run the relevant gates in
   `docs/ai/verification.md` and `pixi run lint` before committing.

## Output

- Original intent, actual outcome, and material session evidence
- Accepted/rejected harness changes, expected future behavior, and owner files
- Validation results, measured benefits or limits, and remaining work
