---
name: dart-execute-packet
description: "DART Execute Packet: select and execute one orchestrator-authored work packet from a numbered plan"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-execute-packet.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-execute-packet

Use this skill in Codex to run the DART `dart-execute-packet` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-execute-packet <arguments>`
- Codex: `$dart-execute-packet <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Execute a work packet in DART: $ARGUMENTS

## Required Reading

Read these files first:
@AGENTS.md
@docs/ai/orchestration.md
@docs/ai/principles.md
@docs/ai/verification.md
@docs/plans/dashboard.md

## Inputs

`$ARGUMENTS` is optional and takes one of three forms:

- `WP-<plan>.<n>` (for example `WP-091.13`) — execute exactly that packet.
- `PLAN-NNN` (for example `PLAN-091`) — select the first available packet in
  that plan.
- empty — auto-select: walk `docs/plans/dashboard.md` top to bottom (document
  order is priority); for each `Active` entry whose owner doc is a numbered
  plan file containing `#### WP-` packet headings, take the first available
  packet by the availability rules below. State which packet was selected and
  why before starting.

If nothing resolves to an available packet, report what was checked (plans
walked, packets skipped and the blocking signal for each) and stop; do not
invent work.

## Availability and conflict check

Run the full check from `docs/ai/orchestration.md` before claiming. A packet
is available only when ALL of these hold:

1. **Dependencies** — its Dependencies line is satisfied in full: every named
   packet is marked `[done — ...]`, and every non-packet precondition (for
   example "maintainer direction on ..." or an accepted design note) has
   recorded evidence in the plan or the named owner doc. Treat any
   precondition you cannot verify as unmet and skip the packet.
2. **Local markers** — its heading carries neither `[done — ...]` nor
   `[claimed]` in the local plan file.
3. **Remote markers** — `git fetch origin` (a read-only sync), then check the
   plan file as it exists on the default branch
   (`git show origin/main:docs/plans/<plan-file>.md`) for a marker the local
   checkout does not have yet.
4. **Branch/PR signals** — no one else is already working it:
   `git ls-remote --heads origin` shows no branch embedding the packet ID in
   the documented form (`wp-<plan>-<n>-<slug>`, for example
   `wp-091-13-contact-assembly`), and `gh pr list --state open --search
"WP-<plan>.<n>"` returns no open PR carrying the ID in its title. These are
   read-only queries that mutate nothing; any push, PR creation, or other
   GitHub mutation still requires explicit maintainer/user approval.

If any signal says the packet is taken: in auto/plan mode skip to the next
available packet; for an explicit packet ID, report the conflicting signal
and stop.

## Readiness check

Before claiming, inspect the packet text and named owner docs for the
specification intake required by `docs/ai/orchestration.md`:

- objective;
- value or rationale;
- scope;
- non-goals;
- assumptions and open decisions;
- acceptance evidence;
- gates; and
- dependencies.

If objective, scope, non-goals, acceptance evidence, gates, or dependencies are
missing or too vague to verify, report that the packet is not executable and
stop. For older packets that lack an explicit value or assumptions field,
proceed only when the owner docs make the value and assumptions unambiguous,
and state those inferred fields before editing. If an unresolved decision would
materially change public API, release compatibility, numerical correctness,
benchmark claims, or roadmap scope, stop and ask the orchestrator to record an
owner-local `Decision needed` block.

## Workflow

1. **Locate the packet** — open the owning numbered plan file linked from
   `docs/plans/dashboard.md` and read the packet's objective, scope,
   value/rationale, assumptions/open decisions, non-goals, acceptance evidence,
   gates, and dependencies.
2. **Claim** — append `[claimed]` to the packet heading in the plan file and
   create the topic branch named `wp-<plan>-<n>-<slug>`. The branch name is
   the cross-machine claim signal once pushed; pushing it (like any GitHub
   mutation) requires explicit maintainer/user approval, so until then the
   marker and branch are local and the strongest remote signal stays the
   merged plan file.
3. **Load packet context** — read the owner docs the plan names for that
   workstream plus the files in the packet's scope. Do not load the whole
   plan corpus; the packet defines the working set.
4. **Implement exactly the packet** — stay inside scope and non-goals. If the
   real scope differs materially from the packet's stated scope, stop and
   report back with what was found; do not widen the packet. One packet, one
   branch, one verification story.
5. **Verify** — run the packet's gates plus `pixi run lint` before any
   commit. Record each piece of acceptance evidence named by the packet
   (test names, command output, doc updates). Missing evidence means the
   packet is not complete — say so explicitly.
6. **Hand back** — append an `Evidence:` bullet to the packet in the plan
   file listing the recorded evidence (or update the dev-task `RESUME.md` for
   multi-session packets), leave the `[claimed]` marker for the orchestrator
   to replace with `[done — ...]` on acceptance, then report completion with
   the evidence list for orchestrator review. Local commits are part of
   execution; pushes and PR creation require explicit maintainer/user
   approval first, and the PR title starts with the packet ID
   (`WP-<plan>.<n>: ...`) so the claim is searchable.

## Rules

- The packet's owner docs win over the packet text on any conflict; report
  the conflict rather than improvising.
- Do not chain into adjacent packets, refactor outside scope, or "fix while
  here" — file findings back to the orchestrator instead.
- Never remove another session's `[claimed]` marker or reuse its branch;
  stale-claim release is the orchestrator's call
  (see `docs/ai/orchestration.md`).
- Behavior-preserving packets must prove preservation (golden trajectories or
  the tests the packet names), not assert it.
- Solver-family work additionally honors the intake checklist in
  `docs/plans/solver-family-intake.md`.
- The author of a packet's implementation does not approve it; acceptance is
  the orchestrator's or an independent reviewer's call.
