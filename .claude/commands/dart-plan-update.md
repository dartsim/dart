---
description: discuss or update DART living plans
argument-hint: "<plan-topic-or-request>"
agent: build
---

Discuss or update DART living plans: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/north-star.md
@docs/plans/README.md
@docs/plans/dashboard.md
@docs/plans/north-star-roadmap.md
@docs/ai/verification.md

## Workflow

1. Classify the request:
   - discussion-only: compare options, priority, scope, or sequencing;
   - plan edit: revise `docs/plans/**` or related indexes;
   - task derivation: turn a plan item into a bounded implementation or docs task.
2. Inspect current evidence before changing plan state. Use repo docs, code,
   tests, CI evidence, issue/PR state, benchmark data, or explicit maintainer
   direction.
   - For new solver/paper implementation plans, preserve the full
     paper-complete bar in `docs/ai/verification.md`: all algorithms/features,
     CPU and GPU paths, paper/site/video demos, benchmark JSON that beats
     reference and paper numbers, and clean long-term API/pipeline work.
   - For active multi-session solver/paper work, update the plan or dev-task
     resume surface with both the completed slice and the next missing
     paper-parity gap; do not let focused tests narrow the recorded objective.
     If the user names source demos, videos, or project pages, keep the corpus
     matrix explicit: which scenes/experiments are represented by tests,
     `py-demos`, visual artifacts, benchmark JSON, CPU reference comparisons,
     and GPU parity, and which rows remain missing.
3. Keep the plan manageable:
   - revise an existing initiative before adding a duplicate;
   - use stable initiative IDs when renaming, splitting, consolidating, or
     parking work;
   - keep `docs/plans/dashboard.md` as the single source of truth for priority,
     status, horizon, dimension, next step, and gate.
   - when deriving packets or dev-task work, include the DART specification
     intake from `docs/ai/orchestration.md`: value, scope, non-goals,
     assumptions/open decisions, acceptance evidence, gates, and dependencies.
     Use owner-local `Decision needed` blocks for consequential ambiguity
     instead of silent defaults.
4. For discussion-only requests, present the tradeoff and proposed plan delta;
   do not edit unless the user asks for an edit or the request already implies
   one.
5. For plan edits, update `docs/plans/dashboard.md` for operating state, the
   detailed numbered initiative file or external owner document for rationale
   and workstreams, and `docs/plans/north-star-roadmap.md` only for strategic
   framing.
6. If the plan item becomes implementation work, route to `/dart-new-task` in
   Claude/OpenCode or `$dart-new-task` in Codex, and use
   `docs/dev_tasks/README.md` when it is multi-session or needs design
   tracking.
7. Verify with `docs/ai/verification.md`: use the docs-only gate for plan-only
   docs, and the AI docs/adapters gate set when AI docs, workflow sources, or
   generated adapters change.
8. Do not perform GitHub or remote mutations without explicit maintainer/user
   approval.

## Output

- Request classification (discussion, plan edit, or task derivation)
- Plan files changed and the operating-state updates made
- Verification gate run
- Any routed follow-up task or `Decision needed` block recorded
