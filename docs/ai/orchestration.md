# Orchestrator / Executor Operating Model

This file owns the DART 6.20 AI work-packet contract. The model is adapted from
DART 7, but the release branch keeps a smaller workflow surface and a stricter
compatibility envelope.

## Roles

DART AI work can split into two roles when the task is large enough:

- **Orchestrator** - owns understanding, decomposition, sequencing, and review.
  The orchestrator reads the north star, the plan dashboard, and relevant owner
  docs; turns roadmap intent into bounded work packets; and reviews executor
  output against acceptance evidence. Large, team-scale, or explicitly
  autonomous release-branch work enters through `dart-ultrawork`, which
  uses `docs/dev_tasks/<task>/` as the project home and starts from either a
  provided brief or one up-front decision interview.
- **Executor** - owns implementation of one packet at a time. The executor
  makes local edits, runs the packet gates, records evidence, and returns.
  Executors do not widen scope or chain adjacent packets without an
  orchestrator decision.

The split is role-based, not tool-based. Every workflow must still map to
tracked docs and `pixi run ...` commands usable without a specific AI tool.

## Review Loop

Review is part of the release-branch work cycle, not a final courtesy pass.
For every meaningful implementation chunk, the orchestrator runs an independent
review lane after verification and again after cleanup or fixes. A packet is
done only after the current post-fix state has at least two clean review passes
recorded in the owning plan, dev-task `verification.md`, or PR evidence.

Use the strongest available reviewer for the risk: `dart-review-pr` for PR
shape and release policy, `dart-analyze` for read-only compatibility or
regression questions, test-focused agents for coverage and failure modes, and
visual reviewers for GUI/example surfaces. When the environment supports
cross-agent review, prefer Codex from a Claude-led workflow, Claude from a
Codex-led workflow, or subagents with disjoint context. If unavailable, perform
a role-separated local review and record the limitation.

Reviewer findings are hypotheses, not commands. Investigate each substantive
finding with code inspection, tests, docs, benchmarks, downstream checks, or
visual evidence; then fix it, split it into a tracked follow-up, or record a
no-fix rationale with evidence. Re-run relevant gates before the next review
pass, and do not mark a packet complete while findings are unexplained.

## Work-Packet Contract

A work packet is the unit of handoff. Packets live in the owning plan file or
dev-task folder, never in a separate tracking system. Every packet records:

- **ID** - stable once published, such as `WP-PG.31`.
- **Objective** - one sentence describing what is true after the packet lands.
- **Value** - the user, maintainer, release, downstream, or research value.
- **Scope** - expected files/modules. A material scope mismatch stops the
  packet.
- **Non-goals** - adjacent work deliberately excluded.
- **Assumptions and open decisions** - defaults inferred from current evidence,
  plus any maintainer decision that blocks execution.
- **Acceptance evidence** - concrete files, tests, benchmark rows, artifacts,
  or command output that prove completion.
- **Gates** - the `pixi run ...` commands from `docs/ai/verification.md`, plus
  packet-specific checks.
- **Dependencies** - packet IDs, merged PRs, or recorded decisions required
  before execution.

For behavior-bearing physics or simulation work, acceptance evidence normally
includes a high-quality, self-contained GUI example or durable demo artifact.
The example should let a user understand the behavior by running and
interacting with it, without reading implementation code or developer notes. If
a GUI artifact is not appropriate on the release branch, the packet must say
why and name the replacement visual or user-level evidence.

## DART 6 Compatibility Checks

Every DART 6 packet that touches code, packages, CI, public docs, or release
workflow must state whether it can affect:

- public headers or installed include paths;
- exported CMake package components;
- ABI-sensitive class layouts or virtual interfaces;
- default collision, constraint, or solver behavior;
- Gazebo/gz-physics or gz-sim downstream behavior;
- dartpy bindings or published user documentation.

If any answer is yes, acceptance evidence needs the matching release-branch
gate. DART 7 clean-break behavior is reference material only.

## Lifecycle

1. The orchestrator drafts or revises packet state in `docs/plans/` or
   `docs/dev_tasks/<task>/`.
2. An executor claims one unblocked packet, creates a topic branch, and keeps
   the branch to that packet's verification story.
3. The executor implements, verifies, and records evidence in the owning
   surface.
4. A separate review pass checks the result against the acceptance evidence.
5. Durable decisions move to `docs/design/`, `docs/onboarding/`,
   `docs/background/`, or `docs/readthedocs/` before a completed dev-task
   folder is removed.

GitHub mutations, pushes, PR creation or updates, CI reruns, review-thread
changes, and merges require explicit maintainer/user approval.
