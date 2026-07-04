---
description: analyze repository evidence without editing
argument-hint: "<question|path|topic>"
agent: build
---

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
