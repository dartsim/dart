---
name: dart-model-upgrade
description: "DART Model Upgrade: audit and improve DART AI harness content and structure for model or coding-agent upgrades, including named models, reasoning modes, migrations, compatibility reviews, and visual simulation-debugging evaluations"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-model-upgrade.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-model-upgrade

Use this skill in Codex to run the DART `dart-model-upgrade` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-model-upgrade <arguments>`
- Codex: `$dart-model-upgrade <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Improve DART's AI harness for: $ARGUMENTS

## Objective

A named model triggers a whole-harness audit: skills, instructions, docs,
context/state ownership, tools, agents, hooks, and verification, including this
workflow — review both content and structure using the audit contract in
`docs/ai/components.md`; updating a model-routing entry is not the whole task.
Later model/effort preferences constrain execution, not this coverage, unless
the user explicitly narrows the audit. Prefer coherent evidence-backed changes:
substantial restructuring is welcome when comparisons show a benefit, while
more machinery or fewer words alone do not establish improvement.

## Required Reading

Read the compact intake first:
@AGENTS.md
@docs/ai/principles.md
@docs/ai/components.md

Then load the relevant sections at the phase that needs them; audit each
surface without loading every referenced document in full:

- Target/control: `docs/ai/README.md` § "Model Routing" and the target tool's
  section in `docs/onboarding/ai-tools.md`.
- Structure/discovery: relevant rows of `docs/ai/workflows.md`, the docs map
  and placement matrix in `docs/README.md`, and `docs/AGENTS.md` before
  documentation changes.
- Project state: current state in `docs/ai/north-star.md`,
  `docs/plans/dashboard.md`, and one representative active plan/handoff;
  `docs/dev_tasks/README.md` and `docs/ai/orchestration.md` own continuation and
  phase-specific authorization. Search long handoffs for current state before
  reading history.
- Comparison/gates: `docs/ai/verification.md` and
  `docs/onboarding/agent-sim-verification.md` with `dart-verify-sim`.
- Tracking/closeout: `docs/dev_tasks/README.md`; load changelog policy only at
  closeout. Use `docs/plans/README.md` only when changing plan state.

## Workflow

1. **Normalize target, boundary, and success.** Preserve the named model,
   reasoning modes, tool version, and branch. Default to `apply` for local
   improvements; honor explicit `audit-only`, discussion-only, and tool Plan
   Mode restrictions. State the boundary, permitted model/effort set, branch,
   and existing authorization. Do not substitute targets or pin the project.
   `audit-only` means no tracked edits, regeneration, dev-task creation,
   auto-fixing lint, commits, pushes, or external mutations. `apply` permits
   local implementation; GitHub mutations still need explicit authorization.
   Reuse supplied decisions; ask only about unresolved consequential choices.
2. **Capture the unchanged control.** Record git state, installed versions,
   `pixi run ai-doctor --json`, authored words/files and declared-reading sizes, skill
   metadata, model/config references, agent inheritance, hooks, scenarios,
   owner/discovery paths, freshness advisories, and focused read-only gates.
   Inspect current state and `docs/dev_tasks/*/RESUME.md` handoffs without
   importing their historical logs by default. Preserve an unchanged snapshot
   outside the changing checkout; keep audit notes and expected answers out of
   evaluation lanes. In multi-session `apply`, create the usual dev-task home.
3. **Refresh primary guidance.** Fetch current official model, prompting,
   migration, configuration, skills, agents, and hook guidance for the target.
   Record URLs, retrieval date, and installed-version evidence. Treat remembered
   limits and repository assumptions as hypotheses. Identify behavior changes
   that call for new evaluation cases, not generic prompt additions.
4. **Audit content and structure; form hypotheses.** Cover every surface in
   `docs/ai/components.md`'s harness audit contract, including the workflow
   itself. Classify each finding: **preserve** (intentional and supported),
   **update** (stale/incorrect), **remove/consolidate** (duplicate or harmful),
   or **add** (a distinct missing responsibility/failure boundary).
   For each proposed change name the observed failure or cost, its owner, the
   candidate change, the comparison that can reject it, and preservation gates.
   Check conflicting stops/approvals, discovery gaps, overloading, stale state,
   repeated facts, and verification that does not cover the actual claim.
   Report removed/merged content and retained owners; distinguish authored
   reductions from generated copies, formatting, and deferred loading. Justify
   net additions and preserved duplication. Do not finish with routing/version
   edits alone unless other surfaces have evidence-backed preserve verdicts.
5. **Compare one variable at a time.** Run fresh non-interactive sessions with
   the same task, tool permissions, physics sources, and input artifacts. Keep
   model, effort, prompt/docs changes, and delegation separate. User restrictions
   bound all lanes and children. Use an older model or lower effort only when
   permitted; otherwise compare the target with unchanged/proposed instructions
   at each authorized setting. Verify recorded parent/child model and effort,
   supply runner settings when the client does not expose them, and inspect
   transcripts for answer leakage, outside reads, and unexpected mutations.
   Record correctness, missing constraints, pauses, context reads, tokens,
   available cost, turns, and wall time. Structural checks and smaller prompts
   alone do not prove model quality; if no behavioral runner is available,
   report structural evidence and the exact limitation instead.
6. **Exercise DART physics and failure boundaries.** Rebuild stale simulation
   artifacts before comparing. Route every target through `dart-verify-sim`:
   seed static overlap hidden by a render, an unstated dynamic rest/tolerance
   criterion, and poor framing offered as trusted. Establish a text oracle
   (metrics, collision/scene/trajectory comparison, or focused behavioral test)
   shown to detect the defect; corroborate it with assessed headless captures
   and only claim-tied debug layers. Require native semantic image inspection,
   repair/rejection of bad views, text/image disagreement handling, and an
   explicit statement of what images do not prove. If rendering or image review
   is unavailable, exercise the text path and `verification-bundle` where
   possible, record the exact limitation, and make no visual-quality claim.
   Also use fresh direct, indirect, incomplete, non-trigger, and edge requests;
   test current-state/resume discovery, supplied decisions, authorization
   carry-forward, unapproved actions, and unavailable requested settings.
   A bare invocation of this workflow must discover the whole-harness audit
   without extra coaching. In `audit-only`, report coverage gaps without edits.
7. **Implement and retest supported changes in `apply` only.** Keep outcome,
   domain constraints, permissions, evidence, and stop conditions explicit.
   Improve owner placement and progressive disclosure; preserve safety and
   public paths through structural changes. Edit `.claude/commands/` and
   `.claude/skills/` sources; run `pixi run sync-ai-commands`, never hand-edit
   generated adapters. Keep model names in `docs/ai/README.md` § "Model Routing"
   and tested-version evidence, not in generic procedures. Follow
   `docs/ai/orchestration.md` for model/effort restrictions and phase-scoped
   delegation. Re-run affected comparisons after meaningful fixes; reject or
   revise candidates that lose required behavior or lack sufficient evidence.
8. **Verify and review the final state.** Run `pixi run check-ai-infra`,
   `pixi run test-ai-infra`, and relevant docs/AI gates from
   `docs/ai/verification.md`, including `pixi run check-docs-policy` for durable
   context changes. Use `pixi run exercise-agent-scenarios` separately when
   scenarios changed or diagnostics are needed; the aggregate already checks
   them. Preserve runner-probe invariants, model-pin boundaries, generated
   parity, and instruction discovery. `audit-only` uses read-only lint such as
   `pixi run check-lint`; `apply` runs `pixi run lint` before any commit.
   Complete the principle audit and two clean role-separated reviews on the
   post-fix state. Label unavailable behavioral, cross-tool, or hosted evidence;
   hosted rate limits do not require waiting after local review converges.
9. **Close out by mode and branch.** `audit-only` stops with findings,
   recommendations, limitations, proposed gates, and apply/adapt/omit advice.
   In `apply`, make the changelog decision, promote durable conclusions to
   their owners, and remove the completed dev-task folder. Report achieved
   outcomes separately from remaining work. Shared infrastructure lands on
   DART 7 first; after an authorized merge, inspect `release-6.20` from its own
   base and record apply/adapt/omit. If parallel release work was explicitly
   requested, keep branches independent and re-audit final main before release
   publication. Never copy main-only paths into the smaller release catalog.

## Output

- Target, branch, mode, authorization, installed versions, sources, and control
- Content/structure coverage with preserve/update/remove/consolidate/add findings
- Hypotheses, accepted/rejected changes, comparison matrix, isolation and limits
- Physics/visual artifacts, text oracles, trigger/failure and resume outcomes
- Mutations, gates, principle audit, two reviews, changelog/cleanup decision
- Local completion, remaining work, and DART 7/DART 6 apply/adapt/omit status
