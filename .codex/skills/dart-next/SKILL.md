---
name: dart-next
description: "DART Next: select and execute the next bounded DART task"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-next.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-next

Use this skill in Codex to run the DART `dart-next` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-next <arguments>`
- Codex: `$dart-next <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Select and execute the next bounded DART task: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/north-star.md
@docs/ai/workflows.md
@docs/ai/verification.md
@docs/plans/README.md
@docs/plans/dashboard.md
@docs/plans/north-star-roadmap.md
@docs/dev_tasks/README.md
@docs/onboarding/contributing.md
@docs/onboarding/ci-cd.md
@docs/onboarding/ai-tools.md

## Arguments

Interpret `$ARGUMENTS` as optional constraints:

- `mode=select`: choose one task and stop with evidence.
- `mode=execute`: choose one task, make local changes, and verify locally.
  This is the default when the user asks to do the work.
- `mode=pr`: execute locally and prepare PR text, then ask for explicit
  maintainer/user approval before any GitHub, PR, CI, branch, or review-thread
  mutation, including pushes, PR creation, PR comments, reviewer requests,
  review re-triggers, thread resolution, ready-for-review transitions, merges,
  CI reruns, or branch deletion.
- `size=tiny|small|medium|large` or `days=N`: fit the chosen task to the
  requested scope. Default to `small`, meaning one focused local session.
- `focus=<topic>`: prefer a focus area without making it the only allowed
  candidate. Examples: `ai-native`, `easy-start`, `algorithm`, `compute`,
  `release`, `ci`, `docs`, `python`, `io`, `PLAN-020`, `world_split`,
  `dartpy`, or a file path.
- `area=<dimension>`: alias for `focus=<dimension>`.
- Any issue, PR, branch, milestone, failing check, file path, or user-stated
  priority overrides the default dashboard order.

If constraints conflict or make the task unsafe to select, stop and ask one
clarifying question.

## Workflow

1. Inspect local state before choosing work:
   ```bash
   git status --short --branch
   git diff --stat
   ```
2. Build the candidate set from tracked evidence:
   - explicit user instruction in `$ARGUMENTS`;
   - current branch, local diff, or active `docs/dev_tasks/<task>/` state;
   - `docs/plans/dashboard.md`, in priority order, especially `Horizon: Now`;
   - `docs/ai/north-star.md` gaps and readiness criteria;
   - issue, PR, or CI state named by the user or inferable from the branch.
3. Exclude candidates that are blocked, larger than the requested size, missing
   enough evidence to start, likely to require unapproved GitHub mutations, or
   only bootstrap/maintain the `dart-next` workflow itself unless explicitly
   focused.
4. Prefer the highest-value remaining candidate in this order:
   - user-specified issue, PR, failing check, or file path;
   - candidates matching `focus=<topic>` or `area=<dimension>`;
   - failing CI or review feedback on the current PR;
   - active dev task with a clear next local step;
   - first unblocked `Horizon: Now` dashboard item fitting the size;
   - issue triage or plan refinement when implementation evidence is missing.
5. State the selected task before editing:
   - candidate and source evidence;
   - rejected higher-priority candidates and why they were not selected;
   - whether the preferred focus matched the selected task, and why if it did
     not;
   - size assumption and expected stopping point;
   - workflow to follow next;
   - verification gate from `docs/ai/verification.md`.

## Execution Routing

After selection, follow the most specific DART workflow:

| Selected work              | Route through                                           |
| -------------------------- | ------------------------------------------------------- |
| New implementation task    | `$dart-new-task` in Codex or `/dart-new-task` elsewhere |
| Existing dev task          | `$dart-resume` in Codex or `/dart-resume` elsewhere     |
| Plan-only change           | `$dart-plan-update` in Codex or `/dart-plan-update`     |
| Docs-only change           | `$dart-docs-update` in Codex or `/dart-docs-update`     |
| GitHub issue fix           | `$dart-fix-issue` in Codex or `/dart-fix-issue`         |
| Downstream bug             | `$dart-downstream-fix` or `/dart-downstream-fix`        |
| Failing CI                 | `$dart-fix-ci` in Codex or `/dart-fix-ci`               |
| PR review or PR management | `$dart-review-pr` / `$dart-manage-pr` or slash versions |

If the selected task is multi-session or needs design tracking, create or
update `docs/dev_tasks/<task>/` according to `docs/dev_tasks/README.md`.

## Local Work Cycle

1. Load task-specific docs from `AGENTS.md` and the routed workflow.
2. Make the smallest coherent local change that satisfies the selected task.
3. Run the strongest relevant local gate:
   - docs/AI work: gates from `docs/ai/verification.md`;
   - C++ or Python behavior: `pixi run lint`, build, and focused tests;
   - CI work: local reproduction when possible plus the fixed check.
4. Review the local diff before declaring completion:
   ```bash
   git diff --stat
   git diff --check
   ```
5. Optimize or benchmark only when the selected objective is performance
   sensitive or the plan gate explicitly requires performance evidence.
6. Update durable docs when the task changes project state, public workflow, or
   roadmap evidence.
7. Run the completion audit from `docs/ai/verification.md` before finalizing.

## PR And Review Management

Use `$dart-pr` or `/dart-pr` only after local verification is complete and the
user requested PR preparation. Ask for explicit maintainer/user approval before
any GitHub, PR, CI, branch, or review-thread mutation, including push, PR
creation, PR comment, reviewer request, review re-trigger, review-thread
resolution, ready-for-review transition, CI rerun, merge, or branch deletion.

After a PR exists and explicit approval covers PR management, use
`$dart-manage-pr` or `/dart-manage-pr` for CI, review, and cleanup. For the
first Codex review on a draft PR, prefer a top-level `@codex review` after
explicit approval for PR comments; it can run while the PR remains draft. Once
Codex is clean and local validation passes on the current head (default
`pixi run test-all`, plus `pixi run -e cuda test-all` on Linux hosts with a
visible NVIDIA CUDA runtime), the draft is ready to mark ready for human review
after approval, while merge still waits for required hosted checks.

## Output

Report:

- selected task, size, evidence, and routed workflow;
- files changed and verification commands run;
- completion-audit result and any remaining blocker;
- PR readiness status, without performing external mutations unless explicit
  approval was given.
