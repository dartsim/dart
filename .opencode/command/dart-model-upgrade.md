---
description: audit and update DART AI infrastructure for model or coding-agent upgrades, including named models, reasoning modes, migrations, compatibility reviews, and visual simulation-debugging evaluations
argument-hint: "<target-model-or-tool-version> [audit-only|apply]"
agent: build
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-model-upgrade.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Audit or update DART's AI infrastructure for: $ARGUMENTS

## Objective

Use live upstream evidence and representative DART tasks to decide what to
preserve, update, remove, consolidate, or add. Prefer the smallest change that
improves the target model or tool without weakening safety, evidence, public
paths, or cross-tool capability parity. Do not equate a larger harness with a
better harness. Keep the reusable intake, comparison, verification, and
closeout core model-agnostic. The workflow itself is an audit surface: improve
it when a target exposes a reusable gap, and replace obsolete target-specific
guidance instead of accumulating one branch or command per model family.
Because DART is a 3D physics simulator, every target must also demonstrate how
well it investigates simulation state with text-first and visual/debug evidence.

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/components.md
@docs/ai/sessions.md
@docs/ai/workflows.md
@docs/ai/verification.md
@docs/AGENTS.md
@docs/README.md
@docs/information-architecture.md
@docs/onboarding/ai-tools.md
@docs/onboarding/agent-sim-verification.md
@docs/dev_tasks/README.md

## Workflow

1. **Normalize the target and scope.** Preserve an explicitly named model,
   reasoning mode, tool version, branch, and `audit-only` or `apply` boundary.
   If no mode is named, state the assumed mode in the first status update and
   keep every mutation local. Record which DART branch is being audited and
   whether GitHub mutations are approved.
   Do not silently substitute a different model or pin the repository to the
   audit target. Treat `audit-only` as read-only for tracked checkout and
   external state: do not edit, regenerate, create a dev-task folder, run
   auto-fixing lint, commit, push, or mutate GitHub. `apply` permits scoped
   local implementation; external mutations still need their own approval.
2. **Capture the control.** Before any apply-mode edit, record `git` state,
   installed tool versions, `pixi run ai-doctor --json`, current model/config
   references, prompt and instruction sizes, generated skill metadata size,
   custom-agent inheritance, hooks, scenarios, durable context and project-state
   owners, active plan/dev-task handoff surfaces, and the branch-local
   `dart-verify-sim` route, rendering/image-evaluation availability, and
   whether its build tree is current (rebuild a stale one before comparing).
   Record docs-policy freshness advisories and baseline read-only focused gates.
   For a multi-session `apply` task, create or refresh `docs/dev_tasks/<task>/`;
   in `audit-only`, only recommend that handoff path.
3. **Refresh primary guidance.** Read the current official model, prompting,
   migration, configuration, skills, agents, and hook guidance relevant to the
   target. Record source URLs, retrieval date, and installed-version evidence.
   Treat repository wording and remembered limits as hypotheses when upstream
   behavior can drift; separate reusable procedure from target-specific
   evidence and flag assumptions in this workflow the new target invalidates.
4. **Classify every finding.** Use these verdicts:
   - **preserve** — current design is intentional and evidence-backed;
   - **update** — guidance or configuration is stale or incorrect;
   - **remove/consolidate** — repeated detail costs context without changing
     behavior, or a rule constrains legitimate work without a principle-backed
     failure it prevents;
   - **add** — a missing trigger, contract, diagnostic, or gate has a distinct
     owner and representative failure it prevents.

   Check model routing and effort, project and custom-agent pins, this workflow
   source and its generated adapters, other workflow sources, `AGENTS.md`
   chains, skill descriptions, tool descriptions, hooks, scenarios, tests, and
   branch-profile differences. Audit the durable context and project-state layer:
   the north star, `docs/ai/sessions.md`, `docs/plans/dashboard.md`, active
   `docs/dev_tasks/*/RESUME.md` handoffs, and the handbook, design, or plan
   owners routed into task sessions. Treat missing discovery, stale state,
   duplicated facts, and excessive default loading as harness findings.

5. **Design a controlled comparison.** Keep model, prompt, configuration,
   reasoning effort, and optional agent features as separate variables. Run
   each lane as a fresh non-interactive session with the same prompt, tool
   allowlist, and checkout state. When the previous model is still served, run
   it as the control; if no old model or behavioral runner is available, use
   structural comparison and say so, and never promote
   structural checks into model-quality claims. Isolate lanes from notes or
   memory written during the audit and confirm from each transcript that no
   lane read them; a lane that saw the expected answer is not a control. When
   access permits, compare:
   - the existing model with existing prompt/settings;
   - the target model with the same prompt and preserved settings;
   - the target at the preserved effort and one lower effort;
   - only then, the smallest justified prompt or configuration change;
   - optional delegation, concurrency, or tool changes in a separate lane.

   For every target, run a representative DART 3D physics investigation through
   the branch's `dart-verify-sim` capability. Give the control and target the
   same scene or behavior claim, seeded so each behavior below can fail: one
   static geometry defect a render hides, one dynamic claim whose rest or
   tolerance threshold is unstated, and one poor framing offered as trusted.
   Require a text correctness oracle such as step metrics, scene/trajectory/
   contact comparison, profiling output, or a focused behavioral test, then
   corroborate it with an assessed headless capture and only the debug layers
   needed by the claim. Record cost, turns, and wall time per lane, and
   compare whether each model:
   - asks for missing evidence instead of guessing;
   - repairs or rejects cropped, occluded, or ambiguous views;
   - selects claim-tied views and debug layers;
   - reconciles text/image disagreement; and
   - states what the image does not prove.

   Images are never the sole correctness oracle. If rendering or image-capable
   review is unavailable, exercise the text path and `verification-bundle`
   where possible, record the exact limitation, and do not turn structural tool
   availability into a model-quality claim.

6. **Route model and reasoning by task shape.** Derive routing for the target
   family from its refreshed official guidance, then record the result in the
   model-routing owner, `docs/ai/README.md` § "Model Routing", which keeps one
   bounded entry per validated tool lane. Match capability tiers to task
   shape: top tier for the hardest ambiguous work, middle tier for everyday or
   read-heavy work, light tier for clear repeatable work. Do not carry one
   family's tier or reasoning-mode names into another family's guidance.
   Deeper reasoning modes give one difficult task more time; parallel lanes
   need explicit user authorization for delegation; most tasks need neither.
   An explicitly requested top-tier evaluation must exercise that lane, not
   turn it into a global default. Replace a stale per-family entry in the
   owner doc; do not clone the workflow or append a section per model family
   here.
7. **Implement only in `apply` mode.** In `audit-only`, skip implementation and
   continue only with non-mutating verification and the evidence report. In
   `apply`, implement the smallest supported delta. Keep outcome, success
   criteria, domain constraints, safety, permissions, evidence, tool routing,
   output, and stop conditions explicit. Remove repeated procedural detail one
   coherent group at a time. Edit `.claude/commands/` or `.claude/skills/`
   sources and regenerate adapters with `pixi run sync-ai-commands`; do not
   hand-edit generated `.agents/skills/` or `.opencode/command/` files. Improve
   owner routing and progressive disclosure instead of loading every plan,
   task, or handbook page by default. Remove superseded model-specific guidance;
   keep model names in the routing owner and tested-version evidence only
   (`pixi run check-ai-infra` enforces it).
8. **Exercise trigger and failure boundaries.** Cover direct, indirect,
   incomplete, non-trigger, and edge prompts as fresh sessions on the target.
   Include a negative case that must retain the existing route, plus
   failure-sensitive checks for model pins, configuration aliases, generated
   parity, instruction discovery, and approval boundaries when touched. Include
   a fresh-session case that must find current project state and the correct
   cross-session resume surface without hidden chat history. Include
   visual-debug failure cases for an unavailable renderer, a poor view,
   text/image disagreement, and a static geometry defect that visual-only
   inspection must not pass. In `audit-only`, assess existing coverage and
   report missing cases without adding them.
9. **Verify and review.** Run focused checks, `pixi run check-ai-infra`,
   `pixi run exercise-agent-scenarios`, `pixi run test-ai-infra`, relevant
   docs/AI checks from `docs/ai/verification.md`, and
   `pixi run check-docs-policy` when the durable context layer is touched.
   Treat gates as evidence, not formality; `docs/ai/verification.md` owns the
   runner-probe invariants the aggregate gate must keep.
   `audit-only` must use read-only lint gates such as `pixi run check-lint`;
   only `apply` runs auto-fixing `pixi run lint` before a commit. Complete the
   principle audit and two clean role-separated reviews on the post-fix state;
   when a hosted review lane is rate-limited, record the converged local
   verification instead of waiting, and label unavailable behavioral,
   cross-tool, or hosted evidence explicitly.
10. **Close out by mode and branch.** In `audit-only`, stop with findings,
    recommendations, limitations, proposed gates, and a branch-local
    apply/adapt/omit recommendation; do not perform change-oriented closeout.
    Only in `apply` mode, make the changelog decision, promote durable guidance,
    and remove the temporary dev-task folder in the completing change. Load
    `docs/onboarding/changelog.md` for that closeout decision instead of
    carrying it through the audit. For shared infrastructure, finish and
    merge DART 7 first, then inspect `release-6.20` from its own base and
    record apply/adapt/omit; on explicit maintainer request for parallel
    release work, keep branches independent and re-audit the final main
    state before the DART 6 PR publishes. Never copy DART 7-only paths or
    assumptions into the intentionally smaller release catalog.

## Output

- Target, branch, mode, installed versions, upstream sources, control state
- Preserve/update/remove/consolidate/add findings with evidence
- Durable context, project-state, session-handoff, and freshness findings
- Comparison matrix with per-lane cost, turns, wall time, and isolation
  evidence; limitations, prompt/config changes, and unchanged choices
- 3D physics investigation and visual/debug evaluation results, artifacts,
  failure-boundary outcomes, and unavailable-evidence limitations
- Direct, indirect, incomplete, non-trigger, and edge-case results
- Mode, mutations performed (none for `audit-only`), gates, principle audit, two
  review passes, and blockers
- For `audit-only`, recommendations and proposed change/validation scope; for
  `apply`, the changelog decision and branch completion state
- For shared changes, the DART 7 then DART 6 apply/adapt/omit verdict
