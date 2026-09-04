---
name: dart-new-task
description: "DART New Task: start a feature, bugfix, refactor, docs, build, or test task"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-new-task.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-new-task

Use this skill in Codex to run the DART `dart-new-task` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-new-task <arguments>`
- Codex: `$dart-new-task <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Start a new task in DART: $ARGUMENTS

## Required Reading

Read these files first:
@AGENTS.md
@docs/ai/principles.md
@docs/information-architecture.md

Then load owners when the task needs them:

- Multi-phase/session work: `docs/dev_tasks/README.md`
  before creating or resuming the project home.
- Branching or contribution: `docs/onboarding/contributing.md` before setup.
- C++/Python/build changes: `docs/onboarding/building.md` and
  `docs/onboarding/code-style.md` before implementation.
- Docs edits: `docs/AGENTS.md` and `docs/README.md` before editing.
- Acceptance and testing: the task's gate set in `docs/ai/verification.md`
  before implementation; load detailed test guidance for the affected surface.

## Workflow

1. **Understand the task** - Parse: goal, constraints, type (feature|bugfix|refactor|docs)
2. **Assess scope** - Multi-phase or multi-session? Create
   `docs/dev_tasks/<task>/` (see `docs/dev_tasks/README.md` for criteria).
   Team-scale work (multiple parallel lanes needing orchestrated worker
   agents) switches to `dart-ultrawork` instead.
   For multi-session, design-heavy, public API, solver/paper, release, or
   cross-module work, fill the dev-task specification intake before editing:
   value, scope, assumptions, traceability, non-goals, acceptance evidence,
   gates, and open decisions. If consequential ambiguity would change public
   API, release compatibility, numerical correctness, benchmark claims, or
   roadmap scope, record an owner-local `Decision needed` block instead of
   silently choosing.
3. **Setup** - Choose the target branch before creating a topic branch:
   - features/docs/non-bugfix refactors: branch from `origin/main`
   - bug fixes that apply to the current release line: branch from the active
     DART 6 LTS `origin/release-6.*` branch first, then cherry-pick or reapply
     to `main`
4. **Implement** - Keep commits focused, follow code style
5. **Verify** - Run `pixi run lint` before committing, then the gate set for
   this task type from `docs/ai/verification.md`. If the claim depends on 3D
   structure or behavior, route through `dart-verify-sim`: text oracle first,
   then assessed claim-tied visual evidence, or record why it is not applicable.
6. **PR** - After explicit maintainer/user approval, `git push -u origin HEAD`
   then `gh pr create --draft --base <target-branch> --milestone "<milestone>"`
   (`DART 7.0` for `main`, branch-matching DART 6.x release milestone for the
   active DART 6 LTS branch); follow `.github/PULL_REQUEST_TEMPLATE.md`
7. **Cleanup** - Before PR: if task used `docs/dev_tasks/<task>/`, first
   promote durable dashboards, evidence matrices, API inventories, migration
   maps, or long-lived decisions into the durable owner selected by
   `docs/information-architecture.md`.
   Then remove the dev-task folder completely (include the deletion in this PR,
   not after merge).

## Type-Specific

- **Bugfix**: Requires PRs to BOTH the active DART 6 LTS branch AND `main`
- **Refactor**: No behavior changes
- **Feature**: Add tests + docs
- **New solver/paper implementation**: record the intake checklist in
  `docs/plans/solver-family-intake.md` first and hold the task to the
  paper-complete bar in `docs/ai/verification.md` § "Research Paper
  Implementation Evidence"; the task is not complete until that bar is met.

## Output

- Task type, scope, and whether a `docs/dev_tasks/<task>/` folder was created
- Files changed and gates run
- Dev-task promotion and cleanup status when the task completed
- PR readiness, noting any external mutation that was explicitly approved
