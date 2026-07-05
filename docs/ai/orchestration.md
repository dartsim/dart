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
  recorded as done. Team-scale tasks enter through `dart-new-team-task`,
  which front-loads a decision interview and routes evidence-resolvable
  uncertainties to spikes and research instead of questions.
- **Executor** — owns implementation of one packet at a time. The executor
  takes a well-defined packet, makes the local edits, runs the packet's
  gates, records the evidence, and hands back. Executors do not widen scope,
  renegotiate the architecture, or chain into adjacent packets without
  returning to the orchestrator. Executors enter through
  `dart-execute-packet`.

The current default role-to-tool assignments live in the Model Routing
section of `docs/ai/README.md`; this file owns only the roles and the
contract.

Authoring and review stay separated: the agent that implemented a packet does
not approve it. The orchestrator (or an independent reviewer session)
performs the acceptance check.

## Work-packet contract

A work packet is the unit of handoff. Packets live inside the owning numbered
plan file (or a dev-task folder for multi-session implementation), never in a
separate tracking system. Every packet records:

- **ID** — `WP-<plan>.<n>` (for example `WP-091.3`), stable once published.
- **Objective** — one sentence; what is true after the packet lands.
- **Value** — the user, maintainer, research, release, or architecture value
  that justifies doing this packet now.
- **Scope** — the files/modules expected to change. An executor finding the
  real scope materially different stops and reports back instead of pushing
  through.
- **Non-goals** — the adjacent work this packet deliberately does not do.
- **Assumptions and open decisions** — defaults inferred from current evidence,
  plus links to any owner-local `Decision needed` block. A packet with a
  consequential unresolved decision is not executable.
- **Acceptance evidence** — the concrete artifacts that prove completion:
  named tests, gate commands, doc updates, benchmark packets. "It compiles"
  is not acceptance evidence.
- **Gates** — the `pixi run ...` commands from `docs/ai/verification.md`
  that must pass, plus packet-specific checks.
- **Dependencies** — packet IDs or merged-evidence preconditions. A packet
  with unmet dependencies is not available for execution.

## Specification intake and readiness

DART uses the existing plan, dev-task, and packet surfaces as its
specification-first workflow. Do not add a parallel `.specify/` tree for normal
DART work. Before implementation starts, the owning surface must answer:

- what value the work delivers;
- what is in scope and explicitly out of scope;
- which assumptions are being made from current evidence;
- which decisions remain open and where they are recorded;
- what acceptance evidence will prove the objective; and
- which `pixi run ...` gates cover that evidence.

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

Specification intake only works when the map matches the territory. Your plan,
context, and prompts are the map; the real code, tests, and requirements are
the territory, and the gap between them is the work's unknowns. Before authoring
or claiming a substantial packet, spend effort converting consequential unknowns
into knowns instead of encoding guesses into a plan. A packet built on
unexplored territory is not ready, however precise its wording looks.

Four DART-native methods, each usable without a specific AI tool:

- **Blind-spot review** — an independent pass, in a different session than the
  one drafting the work, that answers "what is missing, wrong, or unstated
  here?" against the plan and the code it touches. This is the authoring/review
  separation above applied before implementation; route it through
  `dart-analyze` or a read-only reviewer.
- **Throwaway spike** — a time-boxed prototype on a scratch branch whose output
  is knowledge, not merged code. Use it to make the territory visible (does the
  API allow this, where does the real scope land) before the plan hardens, then
  fold the findings back into the packet and discard the spike.
- **Requirements interview** — let the agent interview the maintainer to pull
  out intent, constraints, and non-goals that were never written down, answering
  the intake questions above from the human rather than from a guess. Reserve it
  for consequential ambiguity; short instructions in a well-prepared repo need
  no interview.
- **Reference map** — ground the work in authoritative references (the
  `docs/readthedocs/papers.md` catalog via `dart-references`, design docs, or an
  existing baseline in the code) instead of reconstructing behavior from memory.

Blind-spot review and reference map route through `dart-analyze` and
`dart-references`; the throwaway spike and the requirements interview are
intentionally manual (a scratch branch, a conversation) with no dedicated
workflow. Prefer the lightest method that resolves the unknown, and record what
it resolved as the packet's assumptions-and-open-decisions evidence.

## Sizing rules

Right-sized packets are what make the executor role reliable. The
orchestrator applies these rules when decomposing:

- One packet = one branch = one verification story (the existing
  `docs/ai/north-star.md` bounded-task rule).
- An executor should be able to hold the packet's entire scope in working
  context: prefer a dozen files or fewer; split mechanical sweeps (renames,
  relabels) from judgment work (contract design) into separate packets.
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
   `/dart-execute-packet` (Claude/OpenCode) — appends `[claimed]` to the
   packet heading, and creates the packet's topic branch
   (`wp-<plan>-<n>-<slug>`) so the claim is discoverable by other sessions.
3. The executor implements exactly that packet, runs its gates, records the
   acceptance evidence as an `Evidence:` bullet on the packet (or in the
   dev-task surface for multi-session packets), and reports completion with
   the evidence. Local commits are part of execution; pushes, PR creation,
   and any other GitHub mutation still require explicit maintainer/user
   approval.
4. The orchestrator reviews the result against the packet's acceptance
   evidence, then either replaces `[claimed]` with `[done — <evidence link>]`
   in the plan, returns the packet with findings, or splits follow-up work
   into new packets.
5. When all packets in a workstream close, the orchestrator promotes durable
   output to the owner docs and updates the dashboard per
   `docs/plans/README.md`.

## Packet discovery and claim signals

How any session — human or agent — finds work and avoids collisions:

- **Discovery.** Packets are `#### WP-<plan>.<n>` headings inside numbered
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
     `wp-091-13-contact-assembly`). Executors create this branch at claim
     time; it becomes a cross-machine signal once pushed, and pushing —
     like any GitHub mutation — happens only with explicit maintainer/user
     approval.
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
  wins; report the conflict so the packet (or the doc) is corrected.
- Anything touching shared state beyond the local clone follows the safety
  boundary in `docs/ai/README.md`: GitHub mutations only with explicit
  maintainer/user approval.
- Sibling-lane file collisions: when another lane has an in-flight (open PR)
  change, treat the files in its diff as a do-not-edit set and keep your work
  off them until it merges. If a newly added required gate would trip on that
  lane's pre-existing issues, scope a temporary carve-out (skip those files)
  rather than fixing another lane's code from yours; label the carve-out with an
  explicit removal condition keyed to the sibling lane landing — following the
  `remove_by`/`tracking`/`reason` convention in
  `docs/onboarding/api-boundaries.md` — and remove it once that lane merges.
