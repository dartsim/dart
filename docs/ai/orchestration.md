---
type: ai-operating-model
owner: self
---

# Orchestrator / Executor Operating Model

This file owns DART's two-role AI operating model and the work-packet
contract that connects the roles. `docs/ai/README.md` owns the current model
routing; `docs/ai/workflows.md` owns the workflow catalog;
`docs/ai/verification.md` owns gate selection. The owning numbered plan file
owns active packets themselves (for example
`docs/plans/122-simulation-loop-allocation-hardening.md`), while the dashboard
owns only that plan's operating state.

## Roles

DART AI work splits into two roles. The split is role-based, not tool-based:
any capable agent may fill either role, and every workflow must remain usable
from public docs and `pixi run ...` commands without a specific AI tool.

- **Orchestrator** — owns understanding, decomposition, sequencing, and
  review. The orchestrator reads the north star, the plan dashboard, and the
  relevant design docs; turns roadmap intent into bounded work packets;
  keeps packet scope honest against the simplicity axiom; and reviews
  executor output against the packet's acceptance evidence before the work is
  recorded as done. Team-scale and explicitly autonomous tasks enter through
  `dart-ultrawork`, which uses `docs/dev_tasks/<task>/` as the project
  home, front-loads a decision interview or provided brief, and routes
  evidence-resolvable uncertainties to spikes and research instead of
  questions. Critical decisions, escalations executors cannot resolve, and
  research synthesis use the strongest available reasoning within the user's
  selected model and effort set; routine work stays within that same set.
- **Executor** — owns implementation of one packet at a time. The executor
  takes a well-defined packet, makes the local edits, runs the packet's
  gates, records the evidence, and hands back. Executors do not widen scope,
  renegotiate the architecture, or chain into adjacent packets without
  returning to the orchestrator. Executors enter through
  `dart-execute-packet`.

Current model guidance lives in `docs/ai/README.md`; this file owns the roles
and contract. Codex provides three project-defined read-only profiles:

- `dart_scout` receives an objective, branch, scope, and evidence questions;
  it returns ranked evidence, inferences, and unknowns without edits.
- `dart_reviewer` receives a base/diff, acceptance criteria, and gates; it
  returns severity-ranked findings with proof and missing tests, or a clean
  verdict.
- `dart_release_auditor` receives both branch refs and a proposed common
  change; it returns an apply/adapt/omit matrix plus branch-specific gates.

These profiles isolate context; they do not own implementation. Use built-in
workers or sequential `dart-execute-packet` for mutation, with explicit and
disjoint file ownership whenever parallel writers are used. Custom profiles
inherit the parent model and cannot widen sandbox, approval, or GitHub
authority.

Authoring and review stay separated: the agent that implemented a packet does
not approve it. The orchestrator (or an independent reviewer session)
performs the acceptance check.

## Planning and execution choices

For substantial work, discuss what to build and how to verify it before
implementation. A supplied brief or agreed plan satisfies the decisions it
already resolves; discuss only consequential choices still open. Recommend the reasoning mode by task shape using
`docs/ai/README.md`: independent research questions can benefit from parallel
investigation, while tightly coupled work benefits from one synthesis or
implementation owner. A phase does not by itself select a mode.

Record the reasoning mode, planning/execution phase, rationale, synthesis
owner, and approved delegation scope in the plan or packet's assumptions.
Distinguish research, review, and
implementation permission. Planning-only authorization permits investigation
and expressly requested plan-document updates, not implementation edits. Honor
discussion-only requests and tool Plan Mode restrictions on all file writes.
Research/review delegation does not authorize parallel writers.
Implementation stays sequential unless the user approves independent ownership
scopes. Preserve this decision in the existing resume surface across sessions.

Explicit user model and effort restrictions also apply to children and review
sessions. Verify effective settings instead of assuming a tool inherits effort.
If the requested route is unavailable, report the exact limitation and continue
only work that fits the authorized route; do not silently lower effort or switch
models. Reuse authorization already given for the current scope instead of
asking again. Routine local work needs no extra approval ceremony.

## Review Loop

Review is part of the work cycle, not a final courtesy pass. For every
meaningful implementation chunk, the orchestrator runs an independent review
lane after verification and again after any cleanup or fixes. A packet is done
only when it meets the review-pass item of the completion audit in
`docs/ai/verification.md`, recorded in the owning plan, dev-task
`verification.md`, or PR evidence.

Select a reviewer for the packet's risk within the authorized model and
delegation scope. Prefer an independent session or disjoint context; if that
is unavailable, use a role-separated review and record the limitation.

Reviewer findings are hypotheses, not commands. Investigate each substantive
finding with code inspection, tests, docs, benchmarks, or visual evidence; then
fix it, split it into a tracked follow-up, or record a no-fix rationale with
evidence. Re-run the relevant gates after fixes before asking for the next
review pass. Do not mark a packet complete while review findings are
unexplained, unverified, or only acknowledged.

## Work-packet contract

A work packet is the unit of handoff. Packets live inside the owning numbered
plan file (or a dev-task folder for multi-session implementation), never in a
separate tracking system. Every packet records:

- **ID** — `WP-<plan>.<n>` (for example `WP-122.3`), stable once published.
- **Objective** — one sentence; what is true after the packet lands.
- **Value** — the user, maintainer, research, release, or architecture value
  that justifies doing this packet now.
- **Scope** — the files/modules expected to change. An executor finding the
  real scope materially different stops and reports back instead of pushing
  through.
- **Non-goals** — the adjacent work this packet deliberately does not do.
- **Assumptions and open decisions** — defaults inferred from current evidence,
  reasoning mode and phase, rationale, synthesis/implementation ownership, approved
  delegation phases and scopes, plus links to any owner-local `Decision needed`
  block. A packet with a consequential unresolved decision is not executable.
- **Acceptance evidence** — the concrete artifacts that prove completion:
  named tests, gate commands, doc updates, benchmark packets. "It compiles"
  is not acceptance evidence.
- **Gates** — the `pixi run ...` commands from `docs/ai/verification.md`
  that must pass, plus packet-specific checks.
- **Dependencies** — packet IDs or merged-evidence preconditions. A packet
  with unmet dependencies is not available for execution.

## Specification intake and readiness

Use the work-packet contract above in the existing owning plan or dev-task;
do not create a parallel specification tree. Before implementation, resolve
consequential decisions and name objective, scope, acceptance evidence, gates,
and satisfied dependencies. Vague evidence means the packet is not executable.

When a claim depends on 3D structure or behavior, route verification through
`dart-verify-sim` and record the evidence pairing that
`docs/ai/verification.md` § "GUI And Demo Evidence" requires.

For numbered plans, this information belongs in the work packet. For
multi-session implementation tasks, it belongs in
`docs/dev_tasks/<task>/README.md` and may point back to a plan or design doc.
For small direct changes, it may be implicit in the issue, prompt, and final
verification evidence.

If an orchestrator or executor cannot name the acceptance evidence before
editing, the next step is a planning, clarification, or spike packet (see
"Discovering unknowns before committing" below), not a best-effort
implementation. If a choice would materially change public API,
release compatibility, numerical correctness, benchmark claims, or roadmap
scope, record an owner-local `Decision needed` block instead of silently
choosing.

## Discovering unknowns before committing

Before substantial work, resolve material unknowns with repository inspection,
primary references, a bounded scratch spike, or an independent blind-spot
review. Route read-only analysis through `dart-analyze` and research catalog
work through `dart-references`. Ask the maintainer only for consequential intent
or tradeoffs that evidence cannot settle. Record the resolved evidence and any
remaining owner-local `Decision needed` block in the packet.

## Sizing rules

Right-sized packets are what make the executor role reliable. The
orchestrator applies these rules when decomposing:

- One packet = one branch = one verification story (the existing
  `docs/ai/north-star.md` bounded-task rule).
- An executor should be able to hold the packet's entire scope in working
  context. Split mechanical sweeps from contract design when they need different
  ownership or acceptance evidence.
- Behavior-preserving refactors and behavior-changing fixes never share a
  packet.
- If acceptance evidence cannot be named concretely at authoring time, the
  packet is not ready — it needs a design note or a spike packet first.
- Mechanical multi-file sweeps with known hazards (for example renumbering
  colliding plan IDs) get the hazard list written into the packet, so the
  executor does not rediscover it.

## Lifecycle

1. The orchestrator drafts or revises packets in the owning plan file and
   updates the dashboard entry's next step.
2. An executor picks up the first available packet (see "Packet discovery
   and claim signals" below) — via `$dart-execute-packet` (Codex) or
   `/dart-execute-packet` (Claude Code) — appends `[claimed]` to the
   packet heading, and creates the packet's topic branch
   (`wp-<plan>-<n>-<slug>`) so the claim is discoverable by other sessions.
3. The executor implements exactly that packet, runs its gates, records the
   acceptance evidence as an `Evidence:` bullet on the packet (or in the
   dev-task surface for multi-session packets), and reports completion with
   the evidence. Local commits are part of execution; GitHub mutations follow
   the approval boundary in `docs/ai/principles.md`.
4. The orchestrator reviews the result against the packet's acceptance
   evidence, then either replaces `[claimed]` with `[done — <evidence link>]`
   in the plan, returns the packet with findings, or splits follow-up work
   into new packets.
5. When all packets in a workstream close, the orchestrator promotes durable
   output to the owner docs and updates the dashboard per
   `docs/plans/README.md`.

## Packet discovery and claim signals

How any session — human or agent — finds work and avoids collisions:

- **Discovery.** Packets are `### WP-<plan>.<n>` headings inside numbered
  plan files. Plan priority is `docs/plans/dashboard.md` document order;
  within a plan, packets are taken in document order. A packet's own
  Dependencies line governs availability, and it must be satisfied in full —
  packet IDs marked done plus any non-packet precondition (a maintainer
  decision, an accepted design note) with recorded evidence; an unverifiable
  precondition counts as unmet. Running `dart-execute-packet` with no
  arguments performs exactly this walk and reports its selection.
- **Claim signals**, checked in this order before claiming (fetch from the
  remote first; fetching and listing are read-only):
  1. Markers in the plan file — `[claimed]` / `[done — ...]` on the packet
     heading. The plan file as merged on the default branch is the canonical
     state; the local marker covers sessions sharing a checkout.
  2. A remote branch named `wp-<plan>-<n>-<slug>` (for example
     `wp-122-1-harness-manifest`). Executors create this branch at claim
     time; it becomes a cross-machine signal once pushed, and pushing needs
     explicit maintainer/user approval like any GitHub mutation.
  3. An open PR whose title starts with `WP-<plan>.<n>:` (searchable via the
     GitHub CLI; reading PR lists is read-only and needs no approval).
- **Stale claims.** A `[claimed]` marker or packet branch with no recorded
  evidence and no recent activity is released only by the orchestrator
  (remove the marker or record the packet as returned, with a note).
  Executors never remove another session's claim or reuse its branch.

## Failure and escalation

- Scope mismatch: the executor stops and returns the packet with what was
  found; the orchestrator re-cuts it.
- Gate failure the packet cannot explain: treat as a finding, not a fixup —
  report back rather than patching around it (the packet-scoped case of the
  root-cause axiom in `docs/ai/principles.md`).
- Conflicting instructions between a packet and an owner doc: the owner doc
  wins among repository documents; report the conflict so the packet (or the
  doc) is corrected. This does not override explicit user scope or higher-level
  tool/session instructions.
- Anything touching shared state beyond the local clone follows the safety
  boundary in `docs/ai/principles.md`: GitHub mutations only with explicit
  maintainer/user approval.
- Sibling-lane file collisions: when another lane has an in-flight (open PR)
  change, treat the files in its diff as a do-not-edit set and keep your work
  off them until it merges. If a newly added required gate would trip on that
  lane's pre-existing issues, scope a temporary carve-out (skip those files)
  rather than fixing another lane's code from yours; label the carve-out with an
  explicit removal condition keyed to the sibling lane landing — following the
  `remove_by`/`tracking`/`reason` convention in
  `docs/onboarding/api-boundaries.md` — and remove it once that lane merges.
