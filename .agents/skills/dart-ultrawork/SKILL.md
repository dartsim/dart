---
name: dart-ultrawork
description: "DART Ultrawork: kick off a large or autonomous DART task with project-home docs, an optional decision interview, and orchestrated execution"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-ultrawork.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-ultrawork

Use this skill to run the DART `dart-ultrawork` workflow. The editable workflow
source lives in `.claude/commands/`; this generated skill is its entrypoint for
Codex and Muse.

## Invocation

- Claude Code: `/dart-ultrawork <arguments>`
- Codex: `$dart-ultrawork <arguments>`
- Muse: `/dart-ultrawork <arguments>`

Treat the text after the skill name, or the user's request when the skill is
loaded on demand, as `$ARGUMENTS`. When the workflow references `$1`, `$2`,
etc., map those to the positional values supplied by the user.

## Command Body

Start a team-scale or autonomous DART task: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/ai/principles.md
@docs/ai/north-star.md
@docs/ai/orchestration.md
@docs/ai/sessions.md
@docs/ai/verification.md
@docs/dev_tasks/README.md

## Arguments

`$ARGUMENTS` is a task brief (prose or a structured `TASK` / `CONTEXT` block)
plus optional mode flags:

- `mode=interview`: ask the critical questions up front, one at a time.
- `mode=brief`: treat provided context as sufficient unless escalation applies.
- `mode=resume`: start from the existing `docs/dev_tasks/<task>/` project home
  and run the session-start protocol before changing files.
- `interview=skip`: skip maintainer questions only when the brief already
  answers all consequential decisions.
- `delegation=off`: keep execution and review in the parent for this task,
  as `docs/ai/orchestration.md` defines.

Extract north star, deliverable, acceptance criteria, constraints, risks,
references, paths, issues/PRs/branches, commands, and first step when present.

## Workflow

This workflow is the DART contract layer over the host client's native
autonomous machinery. The client owns the persistence loop, subagents, and
background execution; this workflow owns understanding, decomposition,
sequencing, review, and honest evidence.
Delegate by default under `docs/ai/orchestration.md`, honoring a per-task
`delegation=off` opt-out and keeping small tasks or a single hard workstream
with one agent when proportionate. Use
`dart-new-task` for bounded single-session work unless the user asked
for the autonomous project-home loop.

A work packet is the unit of handoff: one packet = one branch = one
verification story, with an objective, scope, non-goals, acceptance
evidence, and gates named before execution starts. An executor who finds
the real scope materially different stops and reports back.

1. **Session start and current reality** - The project home is
   `docs/dev_tasks/<task>/`, not a parallel project directory. If it exists,
   read `README.md`, `RESUME.md`, and any sidecars (`decisions.md`,
   `verification.md`, `progress-log.md`), verify checkout, branch, and PR
   evidence against them, and refresh stale handoff notes before relying on
   them. If none exists and the task is multi-session, team-scale,
   design-heavy, risky, or explicitly autonomous, create it before
   implementation and absorb any prior work into it. The home is what makes
   the task resumable by a zero-context session in any supported client;
   native transcripts and memories are per-tool and do not replace it.
2. **Understand and scout** - Restate north star, deliverable, acceptance
   criteria, quality bar, non-goals, constraints, and risks. Scout first
   (named docs/code, read-only searches, a `dart-analyze` pass, or
   `dart_scout` in Codex), loading `docs/plans/dashboard.md` and
   `docs/information-architecture.md` for routing only when they apply.
   Draft a candidate decomposition privately before asking anything.
3. **Interview decisions; self-resolve uncertainties** - Split consequential
   unknowns. Ask the human about maintainer decisions (preference, scope,
   public API, release, quality bar, roadmap) up front, one question at a
   time. Give each question its context, 2-4 concrete options with
   tradeoffs, and your recommendation first. Skip these questions under
   `mode=brief`, or under `interview=skip` when the brief already answers
   everything consequential.
   Evidence-resolvable uncertainties (anything a focused A/B test,
   benchmark, spike, reference lookup, or blind-spot review can settle) are
   scheduled as research packets, never asked. Regardless of mode, escalate
   before destructive operations, history rewrites, irreversible
   migrations, meaningful cost, secret handling, legal/privacy-sensitive
   choices, product-direction calls outside the brief, or assumptions whose
   wrong answer could cause significant harm; if input is unavailable, take
   the safest reversible path, document the assumption, and continue only
   with non-blocked work.
4. **Tracking surface** - Populate the project home with value, north star,
   deliverable, scope, non-goals, assumptions, risks, acceptance evidence,
   gates, dependencies, milestone, next actions, and blockers. Keep
   `RESUME.md` as the handoff; add `decisions.md`, `verification.md`, and
   `progress-log.md` when they improve resumability or evidence.
   Claim-dependent physics/simulation, collision/contact, model, Filament,
   and visual-example work routes through `dart-verify-sim`: require a text
   oracle plus assessed visual/debug evidence, or a justified replacement.
5. **Bind the native goal loop** - Express done-when as verifiable outcomes:
   files, tests, gates, and artifacts. Then bind it to the client's native goal
   or continuation mode (`/goal` in Claude Code, a Codex goal, a Muse Code
   session goal). The client then owns keep-working-until-done and stopping:
   do not reimplement that loop in prose. Done means acceptance criteria
   satisfied, verification recorded, docs current, known gaps documented, and
   unnecessary work removed or deferred. Every delegated packet gets its own
   contract: GOAL (one sentence), DONE WHEN (verifiable), EVIDENCE (what to
   record), RISKS, and NEXT STEP.
6. **Decompose, run, review** - Cut packets per the contract above and assign
   each to the best available agent: the parent, native subagents or
   specialists, or another installed supported client. Honor the task's
   delegation opt-out and sequence dependent packets. Give every executor
   explicit, disjoint write ownership;
   implementation stays with the parent or a scoped executor. For each
   meaningful chunk: plan, execute, verify, then run an independent review
   lane (under `delegation=off`, a role-separated local review); treat findings as hypotheses, fix or record no-fix evidence,
   re-verify, and re-review. A packet is not done until the current
   post-fix state has at least two clean review passes recorded. On scope
   mismatch, unblock or re-cut the packet and fold new unknowns back into
   step 3.
7. **Update docs at each stopping point** - Every meaningful cycle updates
   the project home (`README.md` status/plan/risks, `RESUME.md` handoff,
   and the sidecars when they changed) so a zero-context session can resume
   without hidden chat memory.
8. **Closeout** - Keep commits and PRs coherent, separating feature work,
   fixes, refactors, docs, experiments, and AI-infra changes when
   practical. Run `pixi run lint` before commits plus the task-specific
   gates from `docs/ai/verification.md`, record evidence per packet, make
   the changelog decision, and complete the principle audit (load the
   contributing, changelog, and ai-tools owners for external closeout). A
   project completes only when its north star and acceptance criteria are
   met with evidence recorded and docs current; promote durable artifacts
   out of `docs/dev_tasks/<task>/` and remove the folder in the completing
   PR. GitHub mutations only with explicit maintainer/user approval.

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

Put this brief after `/dart-ultrawork` or `$dart-ultrawork`, and make the
same `Done when` list the native goal contract (step 5).

## Output

- Interview record, uncertainty-resolution evidence, and project-home path
- Packet list, routing, goal contracts, gates, and review-loop status
- Per-packet evidence, updated docs, principle audit, cleanup status, and
  approved external mutations
