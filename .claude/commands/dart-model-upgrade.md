---
description: audit and update DART AI infrastructure for model or coding-agent upgrades, including named models, reasoning modes, migrations, and compatibility reviews
argument-hint: "<target-model-or-tool-version> [audit-only|apply]"
agent: build
---

Audit or update DART's AI infrastructure for: $ARGUMENTS

## Objective

Use live upstream evidence and representative DART tasks to decide what to
preserve, update, remove, consolidate, or add. Prefer the smallest change that
improves the target model or tool without weakening safety, evidence, public
paths, or cross-tool capability parity. Do not equate a larger harness with a
better harness.

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/components.md
@docs/ai/workflows.md
@docs/ai/verification.md
@docs/onboarding/ai-tools.md
@docs/dev_tasks/README.md
@docs/onboarding/changelog.md

## Workflow

1. **Normalize the target and scope.** Preserve an explicitly named model,
   reasoning mode, tool version, branch, and `audit-only` or `apply` boundary.
   Record which DART branch is being audited and whether GitHub mutations are
   approved. Do not silently substitute a different model or pin the repository
   to the audit target.
2. **Capture the control.** Before editing, record `git` state, installed tool
   versions, `pixi run ai-doctor --json`, current model/config references,
   prompt and instruction sizes, generated skill metadata size, custom-agent
   inheritance, hooks, scenarios, and baseline focused gates. If the task is
   multi-session, create or refresh `docs/dev_tasks/<task>/`.
3. **Refresh primary guidance.** Read the current official model, prompting,
   migration, configuration, skills, agents, and hook guidance relevant to the
   target. Record source URLs, retrieval date, and installed-version evidence.
   Treat repository wording and remembered limits as hypotheses when upstream
   behavior can drift.
4. **Classify every finding.** Use these verdicts:
   - **preserve** — current design is intentional and evidence-backed;
   - **update** — guidance or configuration is stale or incorrect;
   - **remove/consolidate** — repeated detail costs context without changing
     behavior;
   - **add** — a missing trigger, contract, diagnostic, or gate has a distinct
     owner and representative failure it prevents.

   Check model routing and effort, project and custom-agent pins, workflow
   sources, generated adapters, `AGENTS.md` chains, skill descriptions, tool
   descriptions, hooks, scenarios, tests, and branch-profile differences.

5. **Design a controlled comparison.** Keep model, prompt, configuration,
   reasoning effort, and optional agent features as separate variables. When
   access permits, compare:
   - the existing model with existing prompt/settings;
   - the target model with the same prompt and preserved settings;
   - the target at the preserved effort and one lower effort;
   - only then, the smallest justified prompt or configuration change;
   - optional delegation, concurrency, or tool changes in a separate lane.

   If an old model or behavioral runner is unavailable, use structural
   comparison and say so; never promote structural checks into model-quality
   claims.

6. **Route model and reasoning by task shape.** For the GPT-5.6 family, use Sol
   for the hardest ambiguous work, Terra for everyday or read-heavy work, and
   Luna for clear repeatable work. Max gives one difficult task more reasoning
   time. Ultra is for independently parallelizable work when the user
   authorized delegation. Most tasks need neither. An explicitly requested Sol
   Max evaluation must exercise that lane, not turn it into a global default.
   For another target family, derive routing from its refreshed guidance rather
   than carrying these names forward.
7. **Implement the smallest supported delta.** Keep outcome, success criteria,
   domain constraints, safety, permissions, evidence, tool routing, output, and
   stop conditions explicit. Remove repeated procedural detail one coherent
   group at a time. Edit `.claude/commands/` or `.claude/skills/` sources and
   regenerate adapters with `pixi run sync-ai-commands`; do not hand-edit
   generated `.agents/skills/` or `.opencode/command/` files.
8. **Exercise trigger and failure boundaries.** Cover direct, indirect,
   incomplete, non-trigger, and edge prompts. Include a negative case that must
   retain the existing route, plus failure-sensitive checks for model pins,
   configuration aliases, generated parity, instruction discovery, and
   approval boundaries when touched.
9. **Verify and review.** Run focused checks, `pixi run check-ai-infra`,
   `pixi run exercise-agent-scenarios`, `pixi run test-ai-infra`, relevant
   docs/AI checks from `docs/ai/verification.md`, and `pixi run lint` before a
   commit. Complete the principle audit and two clean role-separated reviews.
   Label unavailable behavioral, cross-tool, or hosted evidence explicitly.
10. **Close out by branch.** Make the changelog decision, promote durable
    guidance, and remove the temporary dev-task folder in the completing
    change. For shared infrastructure, finish and merge DART 7 first; then
    inspect `release-6.20` from its own current base and record an
    apply/adapt/omit verdict. Never copy DART 7-only paths or assumptions into
    the intentionally smaller release catalog. Pushes, PRs, comments, review
    re-triggers, and other GitHub mutations require explicit maintainer/user
    approval.

## Output

- Target, branch, installed versions, upstream sources, and control state
- Preserve/update/remove/consolidate/add findings with evidence
- Comparison matrix, limitations, prompt/config changes, and unchanged choices
- Direct, indirect, incomplete, non-trigger, and edge-case results
- Gates, principle audit, two review passes, changelog decision, and blockers
- Branch completion state and, for shared changes, the DART 7 then DART 6
  apply/adapt/omit verdict
