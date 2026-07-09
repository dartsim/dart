---
name: dart-ultrawork
description: "DART Ultrawork: kick off a large or autonomous DART task with project-home docs, an optional decision interview, and orchestrated execution"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-ultrawork.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-ultrawork

Use this skill in Codex to run the DART `dart-ultrawork` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-ultrawork <arguments>`
- Codex: `$dart-ultrawork <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Start a team-scale or autonomous DART task: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/north-star.md
@docs/ai/orchestration.md
@docs/ai/sessions.md
@docs/plans/dashboard.md
@docs/ai/verification.md
@docs/dev_tasks/README.md
@docs/information-architecture.md
@docs/onboarding/contributing.md
@docs/onboarding/changelog.md
@docs/onboarding/ai-tools.md

## Arguments

`$ARGUMENTS` is a task brief plus optional mode flags:

- `mode=interview`: ask one up-front batch of critical questions.
- `mode=brief`: treat provided context as sufficient unless escalation applies.
- `mode=resume`: start from the existing `docs/dev_tasks/<task>/` project home
  and run the session-start protocol before changing files.
- `interview=skip`: skip maintainer questions only when the brief already
  answers all consequential decisions.

The brief may be prose or a structured `TASK` / `CONTEXT` block. Extract north
star, deliverable, acceptance criteria, constraints, risks, references, paths,
issues/PRs/branches, commands, and first step when present.

## Workflow

You are the orchestrator, supervisor, and steerer for the entire task: you
decompose, delegate, review, and keep evidence honest. Workers implement.
Use `dart-new-task` instead when the work is a bounded single-session task and
the user did not ask for autonomous project handling.

A work packet is the unit of handoff: one packet = one branch = one
verification story, with an objective, scope, non-goals, acceptance
evidence, and gates named before execution starts. An executor who finds
the real scope materially different stops and reports back.

1. **Session start and current reality** - Locate the project home. For
   DART 6 autonomous projects this is `docs/dev_tasks/<task>/`, not a parallel
   project directory. If it exists, read its `README.md`, `RESUME.md`, and any
   autonomous sidecars such as `decisions.md`, `verification.md`, or
   `progress-log.md`; then verify checkout state, current branch, and any
   branch/PR evidence named by the docs. If the docs are stale, update the
   handoff/current-reality note before relying on them. If no project home
   exists and the task is multi-session, team-scale, design-heavy, risky, or
   explicitly autonomous, create `docs/dev_tasks/<task>/` before
   implementation. If prior work exists elsewhere, first absorb or summarize
   it into the project home.
2. **Understand and scout** - Restate the north star, final deliverable,
   acceptance criteria, quality bar, non-goals, constraints, risks, and whether
   the work is release-only or must also land on `main`. Scout the territory
   first with named docs/code, read-only searches, a `dart-analyze` pass, or
   focused reference review; draft a candidate decomposition privately before
   asking anything.
3. **Interview decisions; self-resolve uncertainties** - Ask at most one
   up-front batch of critical questions. Escalate before destructive
   operations, history rewrites, irreversible migrations, meaningful cost,
   security/credential/secret handling, legal or privacy-sensitive decisions,
   major product-direction choices not covered by the brief, conflicts with
   stated constraints, or any assumption whose wrong answer could cause
   significant harm. If input is unavailable, choose the safest reversible path,
   document the assumption, and continue only with non-blocked work. Then split
   consequential unknowns:
   - **Maintainer decisions**: preference, scope, public API, release,
     quality-bar, or roadmap calls that evidence cannot settle. Ask the human
     now in one batched interview (focused questions with 2-4 concrete
     options each, recommendation first). Do not start large work while a
     consequential decision is open. Skip this discretionary interview when
     `mode=brief`; also skip when `interview=skip` and the prompt already
     answers everything consequential. In both cases, still follow the
     escalation rules above.
   - **Evidence-resolvable uncertainties**: anything a focused A/B test,
     benchmark, throwaway spike, reference lookup, or blind-spot review can
     settle. Do not ask the human; schedule these as spike/research packets
     and record the method and result as evidence.
4. **Create or refresh the tracking surface** - Populate the project home with
   value, north star, deliverable, scope, non-goals, assumptions, risks,
   acceptance evidence, gates, dependencies, milestone, next actions, and
   blockers. Behavior-bearing physics/simulation work needs a high-quality,
   self-contained GUI example unless explicitly out of scope. Keep `RESUME.md`
   as the handoff; add `decisions.md`, `verification.md`, and
   `progress-log.md` sidecars when they improve resumability or evidence.
5. **Set the goal contract** - Express done-when as verifiable outcomes
   (files, tests, gates, artifacts). When the session supports a goal or
   stop-hook mode (for example `/goal` in Claude Code), set it to this
   contract so orchestration cannot stop early or loop forever. Stop once the
   acceptance criteria are satisfied, verification is recorded, docs are
   current, known gaps are documented, and unnecessary work has been removed or
   deferred. Every delegated packet gets its own contract: GOAL (one
   sentence), DONE WHEN (verifiable), EVIDENCE (what to record), RISKS, and
   NEXT STEP.
6. **Decompose and route to workers** - Cut work packets per the contract
   above. Route implementation packets to Codex executors, iterative build/test
   lanes to team workers when available, review lanes to an independent
   session, and critical decisions or stuck failures to the oracle. Without
   team tooling, execute packets sequentially in ordinary sessions via
   `dart-new-task`. Use parallelism only when the environment supports it and
   file ownership can stay disjoint.
7. **Run the autonomous work/review cycle** - For each meaningful chunk: plan,
   execute, verify, then run an independent/specialized review lane. Treat
   review findings as hypotheses: investigate, fix or record no-fix evidence,
   clean up, re-verify, and re-review. A packet is not done until the current
   post-fix state has at least two clean review passes recorded.
8. **Supervise and steer** - Monitor progress; unblock, reassign, or re-cut
   packets on scope mismatch. Workers return Task, Summary, Files changed,
   Evidence/tests, Risks, and Recommended next step. Use Codex from Claude,
   Claude from Codex, subagents, or specialist reviewers when available; fall
   back to role-separated local review only when unavailable. Root-cause
   failures and fold newly discovered unknowns back into step 3.
9. **Update docs at each stopping point** - Every meaningful cycle updates the
   project home: `README.md` for status/plan/risks, `RESUME.md` for the next
   fresh-session handoff, `decisions.md` if decisions changed,
   `verification.md` if checks ran or gaps were found, and `progress-log.md`
   for completed chunks when that sidecar exists. Keep docs current enough for
   a zero-context session to resume without hidden chat memory.
10. **Version-control and closeout** - Keep commits and PRs coherent: separate
   feature work, bug fixes, refactors, docs, experiments, and AI-infra changes
   when practical; review the diff, remove unrelated changes, make the
   changelog decision, and run `pixi run lint` before commits. Run
   task-specific gates from `docs/ai/verification.md`, record evidence per
   packet, and complete the principle audit. A project is complete only when
   the north star and acceptance criteria are met, verification evidence is
   recorded, docs are current, known gaps are documented, unnecessary work is
   removed or deferred, and final state is summarized in `RESUME.md` or a
   durable owner. Promote durable artifacts out of `docs/dev_tasks/<task>/`
   and remove the folder in the completing PR. Bug fixes follow the dual-PR
   rule (active DART 6 LTS branch plus `main`). GitHub mutations only with
   explicit maintainer/user approval.

## Kickoff Prompt Template

Canonical fresh-session prompt. `TASK`, context, and `Done when` are per-task;
reuse the `Logistics` block verbatim.

```text
TASK: <one-sentence objective>

<Per-task context: constraints, quality bar, must/never rules, and pointers
to the docs, code, branches, or references that define the territory.>

Done when:
- <verifiable outcome: a file, test, gate, benchmark, or artifact>
- <verifiable outcome>

Logistics:
- Run /dart-ultrawork with this task. You are the orchestrator,
  supervisor, and steerer: decompose, delegate, review, and keep evidence
  honest.
- Interview first: ask the maintainer only the consequential decisions that
  evidence cannot settle; resolve everything else yourself and record the
  evidence.
- Use docs/dev_tasks/<task>/ as the project home. Keep README.md,
  RESUME.md, and any decisions.md / verification.md / progress-log.md
  sidecars current enough for a zero-context session to resume.
- Prioritize correctness over speed, but stop once acceptance criteria are
  satisfied, verification is recorded, docs are current, and known gaps are
  documented. Manage resources responsibly.
- Route well-defined implementation packets to Codex executors with GOAL /
  DONE WHEN / EVIDENCE. Use team mode only when available and file ownership
  can stay disjoint. Keep authoring and review separate. Use the oracle for
  critical decisions, hard failures, and research synthesis.
- Set goal mode to Done-when; use Claude `/goal Run /dart-ultrawork with:
  <real prompt>` or Codex `/goal $dart-ultrawork <real prompt>`. Treat Claude
  goal text beginning `ulw:` or `ultrawok:` as shorthand for the same canonical
  `/dart-ultrawork` workflow, not as separate capabilities.
- Read docs/ai/principles.md, docs/ai/north-star.md,
  docs/ai/orchestration.md, and docs/ai/sessions.md before starting.
- Review loop: use specialized reviewers when available, investigate findings,
  and require two clean passes on the current state before done.
- Verification first: task-specific gates, GUI/demo evidence when relevant,
  pixi run lint before commits; GitHub mutations only with approval.
```

## Output

- Interview record, uncertainty-resolution evidence, and project-home path
- Packet list, routing, goal contracts, gates, and review-loop status
- Per-packet evidence, GUI/demo artifacts when relevant, and updated docs
- Principle audit, cleanup status, and approved external mutations
