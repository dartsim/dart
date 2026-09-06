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

## Codex Read-only Specialists

Current model guidance lives in `docs/ai/README.md`; this file owns the roles
and contract. For a hard single workstream, prefer giving one session more
reasoning over delegating. Use delegated parallel specialists only for
independently parallelizable discovery or review when the user authorized
delegation: `dart_scout` gathers repository evidence,
`dart_reviewer` reviews the current diff and gates, and
`dart_release_auditor` classifies DART 7 material as apply/adapt/omit for this
branch. They inherit the parent model, stay read-only, and must receive explicit
inputs and output contracts. Implementation remains with the parent or a
separately scoped executor; do not give multiple agents overlapping write
ownership.

## Review Loop

Review accompanies meaningful implementation chunks and repairs. Before branch
publication, follow `docs/onboarding/ai-reviews.md`: two distinct non-author
sessions review the immutable candidate, one for correctness and one for
contracts. Supply the complete merge-base diff, acceptance criteria, factual
validation, and prior findings. A strategy checkpoint can serve the contracts
pass when it covers that final candidate; do not add redundant local rounds.

Choose reviewers within the authorized model and reasoning constraints from
`docs/ai/README.md`. Role-separated author investigation can help local work
when independent sessions are unavailable, but cannot clear publication.
Implementation stays with one writer. Findings are hypotheses: inspect their
evidence, repair the underlying family, and rerun gates before obtaining the
new candidate's review pair. Preserve unresolved findings and evidenced
rejections; no clean response silently removes them.

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

When a claim depends on 3D structure or behavior — model/scene loading,
dynamics, collision/contact/constraints, simulation stepping, OSG rendering,
or a visual example — route verification through `dart-verify-sim`. Acceptance
evidence pairs a text correctness oracle with an assessed claim-tied capture
using core bounds/raycast view selection and only the `DebugOverlay` layers
needed by the claim. If visual corroboration is unavailable or genuinely
irrelevant, the packet must say why and name replacement evidence; a screenshot
alone never proves correctness.

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
