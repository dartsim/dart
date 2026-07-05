---
description: kick off a large team-scale DART task with a decision interview, then orchestrate multi-agent execution
argument-hint: "<task description> [interview=skip]"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-new-team-task.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Start a team-scale DART task: $ARGUMENTS

## Required Reading

Read these files first:
@AGENTS.md
@docs/ai/principles.md
@docs/ai/north-star.md
@docs/ai/verification.md
@docs/dev_tasks/README.md

## Workflow

You are the orchestrator, supervisor, and steerer for the entire task: you
decompose, delegate, review, and keep evidence honest. Workers implement.
Use `dart-new-task` instead when the work is a bounded single-session task.

A work packet is the unit of handoff: one packet = one branch = one
verification story, with an objective, scope, non-goals, acceptance
evidence, and gates named before execution starts. An executor who finds
the real scope materially different stops and reports back.

1. **Understand and scout** - Restate the task as objective, quality bar,
   and non-goals. Scout the territory first (read the named docs and code,
   run read-only searches or a `dart-analyze` pass) and draft a candidate
   decomposition privately before asking anything.
2. **Interview decisions; self-resolve uncertainties** - List every
   consequential unknown, then split the list:
   - **Maintainer decisions**: preference, scope, public API, release,
     quality-bar, or roadmap calls that evidence cannot settle. Ask the human
     now in one batched interview (focused questions with 2-4 concrete
     options each, recommendation first). Do not start large work while a
     consequential decision is open. Skip only when `interview=skip` and the
     prompt already answers everything consequential.
   - **Evidence-resolvable uncertainties**: anything a focused A/B test,
     benchmark, throwaway spike, reference lookup, or blind-spot review can
     settle. Do not ask the human; schedule these as spike/research packets
     and record the method and result as evidence.
3. **Create the tracking surface** - Create `docs/dev_tasks/<task>/` with
   `README.md` (specification intake: value, scope, non-goals, assumptions,
   open decisions, acceptance evidence, gates, dependencies) and `RESUME.md`
   (latest completed slice, next gap). Record the interview answers and the
   uncertainty-resolution evidence there. Multi-week scope is expected: plan
   and track well rather than rushing.
4. **Set the goal contract** - Express done-when as verifiable outcomes
   (files, tests, gates, artifacts). When the session supports a goal or
   stop-hook mode (for example `/goal` in Claude Code), set it to this
   contract so orchestration cannot stop early. Every delegated packet gets
   its own contract: GOAL (one sentence), DONE WHEN (verifiable), EVIDENCE
   (what to record).
5. **Decompose and route to workers** - Cut work packets per the contract
   above, then route:
   - **Well-defined implementation packets go to Codex executors, fully
     utilized**: strongest available Codex model at the highest supported
     reasoning effort, write-capable, in goal mode (interactive workers use
     the Codex goal feature; headless runs put GOAL / DONE WHEN / EVIDENCE at
     the top of the prompt), background for long runs. One packet per run
     with scope, non-goals, and gates in the prompt.
   - **Iterative and coordination lanes go to Claude teammates (team mode)**:
     named teammates for build/test iteration and cross-lane coordination,
     with owners pre-assigned and concurrent file ownership kept disjoint
     (worktrees for parallel mutation).
   - **Review lanes stay independent**: a session that implemented a packet
     never approves it.
   - Fallback public path: without Codex or team tooling, execute packets
     sequentially in ordinary sessions via `dart-new-task`; the packet
     contract is unchanged.
6. **Supervise and steer** - Monitor progress; unblock, reassign, or re-cut
   packets on scope mismatch. Review every returned packet against its
   acceptance evidence before recording it done. Root-cause failures instead
   of patching around them; fold newly discovered unknowns back into step 2.
7. **Verify and close** - Run the task-specific gates from
   `docs/ai/verification.md` and `pixi run lint` before commits; record
   evidence per packet. Complete the principle audit. Promote durable
   artifacts out of `docs/dev_tasks/<task>/` and remove the folder in the
   completing PR. Bug fixes follow the dual-PR rule (active DART 6 LTS branch
   plus `main`). GitHub mutations (push, PR, comments, re-triggers) only with
   explicit maintainer/user approval.

## Kickoff Prompt Template

Canonical prompt for starting this workflow from a fresh orchestrator
session. `TASK`, the context block, and `Done when` are per-task; reuse the
`Logistics` block verbatim.

```text
TASK: <one-sentence objective>

<Per-task context: constraints, quality bar, must/never rules, and pointers
to the docs, code, branches, or references that define the territory.>

Done when:
- <verifiable outcome: a file, test, gate, benchmark, or artifact>
- <verifiable outcome>

Logistics:
- Run /dart-new-team-task with this task. You (the strongest available
  Claude model) are the orchestrator, supervisor, and steerer: decompose,
  delegate, review, and keep evidence honest; do not implement large
  packets yourself.
- Interview first: ask the maintainer only the consequential decisions that
  evidence cannot settle; resolve everything else yourself with focused A/B
  tests, benchmarks, throwaway spikes, and research, and record the
  evidence.
- Fully utilize Codex for well-defined packets: strongest available Codex
  model, highest supported reasoning effort, write-capable, goal mode, one
  packet per run with GOAL / DONE WHEN / EVIDENCE plus scope, non-goals,
  and gates in the prompt; background for long runs.
- Use team mode for parallel lanes: named Claude teammates for iterative
  build/test and coordination work; keep authoring and review lanes
  separate; keep concurrent file ownership disjoint.
- Set the session goal (/goal) to the Done-when contract so orchestration
  cannot stop early.
- Read docs/ai/principles.md and docs/ai/north-star.md before starting;
  follow the work-packet contract in dart-new-team-task.
- Large scope is expected: do not optimize for hours or days; plan well and
  track everything in docs/dev_tasks/<task>/ (README.md intake, RESUME.md).
- Verification first: task-specific gates from docs/ai/verification.md,
  pixi run lint before commits, evidence per packet; GitHub mutations only
  with explicit maintainer/user approval.
```

## Output

- Interview record: decisions asked with answers, plus uncertainties routed
  to spike/research packets with their resolution evidence
- Dev-task folder path, packet list with routing (Codex executor or Claude
  teammate) and per-packet goal contracts
- Per-packet acceptance evidence and gate results as work completes
- Principle-audit result, dev-task promotion and cleanup status, and any
  external mutation that was explicitly approved
