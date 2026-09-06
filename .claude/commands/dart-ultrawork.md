---
description: kick off a large or autonomous DART task with project-home docs, an optional decision interview, and orchestrated execution
argument-hint: "<TASK/CONTEXT> [mode=interview|brief|resume] [interview=skip]"
agent: build
---

Start a team-scale or autonomous DART task: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/north-star.md
@docs/ai/orchestration.md
@docs/dev_tasks/README.md
@docs/ai/verification.md

Load additional owners only when the matching phase needs them:

- placement or cleanup: `docs/README.md`;
- numbered-plan selection or packet state: `docs/plans/dashboard.md`;
- version control, changelog, tools, or review:
  `docs/onboarding/{contributing,changelog,ai-tools}.md`.

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

Own understanding, decomposition, sequencing, review, and honest evidence for
the whole task. Follow the orchestrator/executor and packet-sizing contracts in
`docs/ai/orchestration.md`. Delegate only when the user explicitly requested it
and the current surface permits it; otherwise execute packets serially. Use
`dart-new-task` for bounded single-session work unless the user asked for the
autonomous project-home loop.

1. **Session start and current reality** - Follow `docs/dev_tasks/README.md`'s
   Session Start protocol for the `docs/dev_tasks/<task>/` project home:
   current snapshot and next action first, history only as needed, then verify
   live branch/PR/plan state before acting. Run `pixi run ai-doctor` when setup,
   discovery, instruction, agent, or hook state is uncertain. Create or refresh
   the project home before implementation when the session policy requires it.
2. **Understand and scout** - Restate the north star, final deliverable,
   acceptance criteria, quality bar, non-goals, constraints, risks, and target
   branch line (DART 7 `main`, DART 6 LTS, or both). Scout the territory first
   with named docs/code, read-only searches, a `dart-analyze` pass, the Codex
   `dart_scout` profile, or focused reference review; draft a candidate
   decomposition privately before asking anything.
3. **Interview decisions; self-resolve uncertainties** - Ask at most one
   up-front batch of critical questions, only for choices or authority missing
   from the brief and prior decisions. Escalate before destructive
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
     options each, recommendation first). Defer work that depends on an open
     decision; continue independent work already authorized. Skip this discretionary interview when
     `mode=brief`; also skip when `interview=skip` and the prompt already
     answers everything consequential. In both cases, still follow the
     escalation rules above.
   - **Evidence-resolvable uncertainties**: anything a focused A/B test,
     benchmark, throwaway spike, reference lookup, or blind-spot review can
     settle. Do not ask the human; schedule these as spike/research packets
     and record the method and result as evidence (see "Discovering unknowns
     before committing" in `docs/ai/orchestration.md`).
4. **Create or refresh the tracking surface** - Populate the project home with
   value, north star, deliverable, scope, non-goals, assumptions, risks,
   acceptance evidence, gates, dependencies, milestone, next actions, and
   blockers. Claim-dependent 3D structure or behavior work routes through
   `dart-verify-sim`.
   Keep `RESUME.md` as the handoff; add `decisions.md`, `verification.md`,
   and `progress-log.md` sidecars when they improve resumability or evidence.
5. **Set the goal contract** - Express done-when as verifiable outcomes
   (files, tests, gates, artifacts). Activate a persistent goal or stop-hook
   mode only when the user explicitly requests it and the tool supports it.
   Stop once the
   acceptance criteria are satisfied, verification is recorded, docs are
   current, known gaps are documented, and unnecessary work has been removed or
   deferred. Every delegated packet gets its own contract: GOAL (one
   sentence), DONE WHEN (verifiable), EVIDENCE (what to record), RISKS, and
   NEXT STEP.
6. **Decompose and route** - Cut work packets per `docs/ai/orchestration.md`
   and route by `docs/ai/README.md`. Execute serially by default. When the user
   explicitly requested delegation, use a read-only scout for territory
   mapping, bounded workers or `dart-execute-packet` for implementation, an
   independent reviewer for acceptance review, and a release auditor for
   branch adaptation; Codex supplies these roles as the `.codex/agents/`
   profiles and other tools use separate sessions. Use parallel writers only
   with user-approved implementation delegation and explicit disjoint ownership;
   research/review approval alone is insufficient. Record the phase-specific
   mode and delegation decision per `docs/ai/orchestration.md`.
7. **Run the autonomous work/review cycle** - For each meaningful chunk: plan,
   execute, verify, then run an independent/specialized review lane. Treat
   review findings as hypotheses: investigate, fix or record no-fix evidence,
   clean up, re-verify, and re-review. A packet is not done until the current
   post-fix state has at least two clean review passes recorded.
8. **Supervise and steer** - Monitor progress; unblock, reassign, or re-cut
   packets on scope mismatch. Workers return Task, Summary, Files changed,
   Evidence/tests, Risks, and Recommended next step. Use another tool, an
   independent session, or the bounded specialist profiles within the approved
   model/effort and delegation scope; use role-separated local review when an
   independent route is unavailable under that scope. Root-cause
   failures and fold newly discovered unknowns back into step 3.
9. **Update docs at each stopping point** - Follow `docs/dev_tasks/README.md`'s
   Session End protocol. Keep the current snapshot sufficient for a fresh
   session to resume without hidden chat memory or reading the entire history.
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
    and remove the folder in the completing PR. GitHub mutations (push, PR,
    comments, re-triggers) only with explicit maintainer/user approval.

## Prompt Shape

Use an outcome-first brief. Do not repeat this workflow's logistics or required
reading in the task prompt; the capability loads them.

```text
TASK: <one-sentence objective>

Done when:
- <verifiable outcome: a file, test, gate, benchmark, or artifact>
- <verifiable outcome>

Constraints/evidence:
- <task-specific must/never rules and owner references>
- <known risks, branch/PR facts, or required comparison>
```

Put this brief after `/dart-ultrawork` or `$dart-ultrawork`. When goal mode is
available, make the same `Done when` list the goal contract.

## Output

- Interview record, uncertainty-resolution evidence, and project-home path
- Packet list, routing, goal contracts, gates, and review-loop status
- Per-packet evidence, GUI/demo artifacts when relevant, and updated docs
- Principle audit, cleanup status, and approved external mutations
