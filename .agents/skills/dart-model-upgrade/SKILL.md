---
name: dart-model-upgrade
description: "DART Model Upgrade: audit and update DART 6 AI infrastructure for model or coding-agent upgrades, including named models, reasoning modes, migrations, compatibility reviews, and visual simulation-debugging evaluations"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-model-upgrade.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-model-upgrade

Use this skill in Codex to run the DART `dart-model-upgrade` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-model-upgrade <arguments>`
- Codex: `$dart-model-upgrade <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Audit or update DART 6.20's AI infrastructure for: $ARGUMENTS

## Objective

Use live primary guidance and representative release-branch tasks to decide
what to preserve, update, remove, consolidate, or add. Prefer the smallest
change that improves the target without weakening DART 6 compatibility,
safety, evidence, public paths, or cross-tool capability parity. The reusable
intake, comparison, verification, and closeout core stays model-agnostic. This
workflow is itself an audit surface: improve it when a target exposes a
reusable gap, and replace stale target-specific guidance instead of
accumulating one command or permanent branch per model family. Because DART is
a 3D physics simulator, every target also demonstrates how well it investigates
simulation state with text-first and OSG visual/debug evidence.

## Required Reading

@AGENTS.md
@docs/AGENTS.md
@docs/README.md
@docs/information-architecture.md
@docs/ai/principles.md
@docs/ai/README.md
@docs/ai/components.md
@docs/ai/sessions.md
@docs/ai/workflows.md
@docs/ai/verification.md
@docs/ai/branch-profile.json
@docs/onboarding/ai-tools.md
@docs/onboarding/release-management.md
@docs/dev_tasks/README.md

## Workflow

1. **Normalize target, mode, and authority.** Preserve the named model,
   reasoning mode, tool version, branch, and `audit-only` or `apply` boundary.
   Record whether external mutations are approved. Do not substitute a model
   or pin the repository to the audit target. `audit-only` is read-only for
   tracked files and external state: do not edit, regenerate, create task
   state, run auto-fixing lint, commit, push, or mutate GitHub. `apply` permits
   scoped local implementation; external mutations still need approval.
2. **Capture the release control.** Before apply-mode edits, record `git`
   state, installed versions, `pixi run ai-doctor --json`, model/config
   references, prompt and instruction sizes, generated skill-metadata size,
   custom-agent inheritance, hooks, scenarios, durable context and project
   state, active plan/dev-task handoffs, and the branch-local
   `dart-verify-sim`/OSG/image-evaluation route. Run baseline read-only gates.
   Create `docs/dev_tasks/<task>/` only when the apply task is genuinely
   multi-session; audit-only reports the proposed path without creating it.
3. **Refresh primary guidance.** Read current official model, prompting,
   migration, configuration, skills, agents, and hook guidance relevant to the
   target. Record URLs, retrieval date, and installed-version evidence.
   Separate reusable procedure from target-specific evidence and flag any
   assumption in this workflow that the target invalidates.
4. **Classify every finding.** Use these verdicts:
   - **preserve** — intentional and evidence-backed;
   - **update** — stale or incorrect guidance/configuration;
   - **remove/consolidate** — repeated detail costs context without changing
     behavior;
   - **add** — a missing trigger, contract, diagnostic, or gate has a distinct
     owner and representative failure it prevents.

   Inspect model/effort routing, project and custom-agent pins, this workflow
   and generated adapters, other prompts, `AGENTS.md` chains, descriptions,
   hooks, scenarios, tests, and `docs/ai/branch-profile.json`. Audit AI
   infrastructure as both tooling and durable project context: the north star,
   `docs/ai/sessions.md`, `docs/plans/dashboard.md`, active
   `docs/dev_tasks/*/RESUME.md`, and handbook/design/release owners routed into
   sessions. Check discovery, freshness, duplication, context cost,
   cross-session resume quality, and usefulness to humans as well as agents.
5. **Design a controlled comparison.** Keep model, prompt, configuration,
   reasoning effort, and optional agent features as separate variables. When
   access permits, compare the existing model/settings, the target with the
   same prompt and preserved effort, the target at one lower effort, then only
   the smallest justified prompt/config change. Test delegation or concurrency
   separately. If a behavioral runner is unavailable, report structural
   evidence without turning it into a model-quality claim.

   Every target runs one representative DART 6 physics investigation through
   `dart-verify-sim` using the same scene and claim. Require a text correctness
   oracle, then corroborate it with an assessed OSG capture and only the needed
   `DebugOverlay` layers. Compare whether the model requests missing evidence,
   repairs poor views, chooses claim-tied views/layers, reconciles text/image
   disagreement, and says what the image does not prove. Images are never the
   sole correctness oracle. If rendering or native image review is unavailable,
   exercise the text path, record the exact limitation, and do not infer visual
   quality from tool availability.
6. **Route by task shape.** For GPT-5.6, use Sol for difficult ambiguous work,
   Terra for everyday or read-heavy work, and Luna for clear repeatable work.
   Max gives one hard task more reasoning time. Ultra is for independently
   parallelizable work only when the user authorized delegation. Most tasks
   need neither. An explicitly requested Sol Max evaluation exercises that
   lane without making it a project default. For another family, derive
   routing from refreshed guidance and replace this bounded example when stale;
   do not clone the workflow or append an ever-growing model taxonomy.
7. **Implement only in apply mode.** Apply the smallest supported delta.
   Preserve C++17, pybind11, `dart::utils`, OSG, public/ABI/package behavior,
   and Gazebo/gz-physics compatibility. Do not import DART 7-only C++23,
   nanobind, `dart::io`, solver/backend, renderer, or plan surfaces. Keep
   outcome, constraints, permissions, evidence, routing, and stopping
   conditions explicit. Edit `.claude/` sources and run
   `pixi run sync-ai-commands`; never hand-edit generated adapters. Prefer
   progressive disclosure over loading every plan or task by default. Improve
   this source in the same apply change when evidence exposes a reusable gap,
   and remove superseded model-specific guidance.
8. **Exercise trigger and failure boundaries.** Cover direct, indirect,
   incomplete, non-trigger, and edge prompts. Include a negative case that
   retains its existing route plus failures for model pins, config aliases,
   generated parity, instruction discovery, approvals, unavailable OSG,
   rejected views, text/image disagreement, and a static geometry defect that
   visual-only inspection must not pass. Include a fresh-session case that
   finds current project state and the correct resume surface without hidden
   chat history. Audit-only assesses and reports missing cases without adding
   them.
9. **Verify and review.** Run focused checks, `pixi run check-ai-infra`,
   `pixi run exercise-agent-scenarios`, `pixi run test-ai-infra`, the relevant
   docs/AI gates in `docs/ai/verification.md`, and the representative OSG
   investigation. Treat the configured CMake File API result, expanded
   target-command trace, and CTest inventory from `check-ai-infra` as the
   effective test-graph proof. Confirm inactive tests use explicit reached
   predicates, CTest registrations execute rather than list, and pytest remains
   pinned to the trusted root configuration and pre-import package provenance;
   source-marker matches alone are insufficient.
   Audit-only uses `pixi run check-lint`; only apply mode runs auto-fixing
   `pixi run lint` before a commit. Complete the principle audit and two clean
   role-separated reviews. Label unavailable behavioral, cross-tool, hosted,
   or downstream evidence.
10. **Close out for DART 6.** Make the changelog decision in apply mode,
    promote durable guidance, and remove completing temporary task state.
    Compare any related DART 7 PR or commit against the current release base
    and record apply/adapt/omit for every material surface. Normally land
    shared work on `main` first; when a maintainer explicitly requests parallel
    release work, keep the branches independent and re-audit the final main
    state before the DART 6 PR is published or merged. Never copy DART 7-only
    assumptions into this intentionally smaller catalog. Pushes, PRs,
    comments, review triggers, CI reruns, and merges require explicit
    maintainer/user approval.

## Output

- Target, mode, branch, installed versions, primary sources, and control state
- Preserve/update/remove/consolidate/add plus DART 7 apply/adapt/omit verdicts
- Durable context, session-handoff, freshness, and context-cost findings
- Comparison matrix, limitations, prompt/config changes, and unchanged choices
- DART 6 physics/OSG investigation, semantic visual review, artifacts, and
  failure-boundary results
- Gates, compatibility effects, principle audit, two reviews, and blockers
- In apply mode, the changelog decision and local/external completion state
