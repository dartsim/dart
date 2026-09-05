# DART Living Plans

This directory holds DART's living plan from current state to the AI-native
north star. Use it when deciding what to do next, revising priorities, or
turning a strategic direction into bounded work.

## Why `docs/plans/`

`docs/plans/` is a collection of living planning docs, so it follows the
plural collection style used by `docs/dev_tasks/`. It is separate from
`docs/dev_tasks/`. For cross-bucket placement decisions, use
[`docs/information-architecture.md`](../information-architecture.md).

The plan-specific split is:

- `docs/plans/` records the current strategic path, priority order, open gaps,
  gates, and acceptance criteria.
- `docs/design/` records durable design proposals and decision rationale that
  should outlive a particular roadmap sequence.
- `docs/dev_tasks/` tracks active multi-session implementation work and is
  deleted when that work completes.
- `docs/onboarding/` keeps durable developer design explanations after work
  lands.

## Lifecycle

Plans are living execution surfaces, not a permanent knowledge base or an
archive. A plan should help maintainers and agents decide what to tackle next,
then shrink or disappear after its durable output moves to the right owner.
Use the placement matrix in
[`docs/information-architecture.md`](../information-architecture.md) when
choosing that durable owner.

Use this lifecycle:

1. Create or update a numbered plan when a roadmap gap needs priority,
   sequencing, gates, and acceptance criteria.
2. Derive bounded work from the plan. Use a direct PR for small work, or
   `docs/dev_tasks/<task>/` when implementation needs multi-session tracking.
3. Move durable output out of the plan:
   - user instructions go to `docs/readthedocs/` or `README.md`;
   - developer knowledge goes to `docs/onboarding/`;
   - durable architecture, API shape, and tradeoff rationale go to
     `docs/design/`;
   - release and compatibility facts go to release, changelog, or
     compatibility owner docs.
4. When a plan completes (durable owner docs, examples, tests, or code hold the
   result), move its entry out of `dashboard.md` and into
   [`archive.md`](archive.md) in the same PR, converting it to the
   `**Final status:** Complete` shape. The dashboard shows only operating
   (non-`Complete`) plans; archived entries point to durable owner docs, not to
   an archival numbered plan file.
5. Remove or consolidate numbered plan files once they no longer guide current
   prioritization. Git history preserves old plan text; `docs/plans/` should
   keep moving.

Create a follow-up plan only when new roadmap work remains. Avoid appending
"next phase" to the same initiative forever.

## Living Plans vs Design Docs

`docs/plans/` owns time-variant roadmap state: priority, status, horizon, next
step, gates, open gaps, sequencing, and acceptance criteria. Plans describe the
current intended path.

`docs/design/` owns durable technical reasoning: architecture, API shape,
tradeoffs, constraints, and decision rationale. Design docs may be revised, but
they should not own priority, timeline, or active implementation state.

## Files

Do not maintain a per-file inventory here; it drifts as plans land and archive.
List the current set with `ls docs/plans/` and read files by role:

- `dashboard.md` owns operating priority, status, next step, and gate per plan;
  `archive.md` holds completed entries.
- `north-star-roadmap.md` owns strategic framing; `solver-family-intake.md`
  owns the intake checklist for new solver families.
- `NNN-<initiative>.md` files own one initiative's workstreams, acceptance
  criteria, and progress log; a matching `NNN-<initiative>/` directory holds
  that initiative's sidecars (audits, matrices, manifests, roadmaps).
- `AGENTS.md` carries the folder-local agent rules; `../ai/north-star.md` and
  `../ai/verification.md` are the mission and gate owners this folder serves.
- External owner docs are linked from `dashboard.md` entries.

## Directory Structure

Keep `docs/plans/` flat for the dashboard, roadmap, and single-file initiative
plans. Flat files are easier for maintainers and agents to scan, link, diff,
and reorder while the plan set is small enough to fit in the dashboard.

Use a sidecar subdirectory only when one active initiative needs multiple
planning artifacts, such as an evidence matrix, benchmark decision record, API
inventory, or migration map. Keep the stable `.md` owner file in place and add
the sidecar directory under the same initiative ID:

```text
docs/plans/NNN-active-initiative.md
docs/plans/NNN-active-initiative/
├── baseline-harness.md
└── api-inventory.md
```

Only move the owner file into a directory when the plan truly outgrows the
single-file shape. If that happens, use `git mv` and update every owner link in
the same change.

Do not create subdirectories by status, horizon, or north-star dimension.
Plans move across those categories often, and directory moves make links and
diffs noisier. Implementation tracking still belongs in `docs/dev_tasks/`, not
in `docs/plans/`.

## Single Source Of Truth

Avoid duplicating tracking fields across files. `dashboard.md` owns plan
priority, status, horizon, north-star dimension, next step, and gate. Detailed
plan files and external owner documents should point to the dashboard for those
fields instead of repeating them.

Dashboard entries may link to either:

- a detailed numbered initiative file in `docs/plans/`; or
- an authoritative owner document, such as a release roadmap or active dev-task
  design, when that surface already owns the scope and a dedicated plan file
  would duplicate it.

Keep dashboard entries git-history friendly: one plan per block, priority order
by document order, and one frequently changed field per line.

Keep the dashboard a bounded operating view. Each `### PLAN-` block stays at or
under 40 lines and its `- Next step:` field at or under 15 lines. When an entry
outgrows that budget, relocate the historical or evidence narrative into the
owner numbered plan file under a `## Progress log` section and leave only the
current next action plus a `History:` pointer in the dashboard. Completed plans
leave the dashboard entirely for [`archive.md`](archive.md).

When editing:

- update `dashboard.md` for operating state;
- update the detailed numbered initiative file or external owner document linked
  from `dashboard.md` for scope, workstreams, acceptance criteria, revision
  triggers, and rationale;
- update `north-star-roadmap.md` only when strategic framing or sequencing
  principles change.

## Planning Principles

Plans should be:

- **Clearly designed**: each initiative has a named outcome, owner surface,
  next decision, and verification evidence.
- **Frequently revised**: priority order, horizon, scope, and initiative shape
  should change as evidence changes.
- **Current-state oriented**: describe the intended path now, not every previous
  path considered.
- **Small enough to maintain**: keep initiative cards concise and move detailed
  design into developer docs or active dev tasks.
- **Evidence-backed**: every change should cite repository docs, code, CI,
  issue/PR state, benchmark data, or explicit maintainer direction.
- **Cleaned up when complete**: remove, consolidate, or retarget plan files
  after durable output lands in `docs/readthedocs/`, `docs/onboarding/`,
  `docs/design/`, code, tests, examples, or release docs.

## Revision Triggers

Revise plans when any of these happen:

- the north star changes;
- a release priority changes;
- CI, benchmarks, packaging, or downstream evidence changes risk or sequencing;
- a new research direction needs an algorithm extension point or baseline;
- a plan item is split, consolidated, removed, completed, blocked, or parked;
- a dev task starts or completes and changes roadmap state;
- a maintainer asks to compare alternatives before changing direction.
- the dashboard stops being useful as an operating view (the entry and
  next-step size budgets are enforced by the structural checks, so relocate any
  overflow to the owner plan file's `## Progress log` and move completed entries
  to `archive.md`).
- a completed plan still points at a numbered plan file instead of a durable
  owner doc.

## Initiative Card Shape

Use this shape for plan entries:

```markdown
### PLAN-000: Short Name

- Operating state: `PLAN-000` in `docs/plans/dashboard.md`
- Outcome: <what success looks like>
- Current evidence: <repo docs, code, tests, CI, issue/PR state, benchmark data>
```

Statuses mean:

| Status   | Meaning                                            |
| -------- | -------------------------------------------------- |
| Proposed | Worth considering but not actively sequenced       |
| Active   | Currently part of the intended path                |
| Blocked  | Valuable but waiting on a named decision/evidence  |
| Complete | Outcome achieved; durable output lives elsewhere   |
| Parked   | Intentionally deferred; revisit only with new data |

Horizons mean:

| Horizon | Meaning                                                          |
| ------- | ---------------------------------------------------------------- |
| Now     | Highest-priority work for the next planning/implementation cycle |
| Next    | Sequenced after current blockers or active tasks clear           |
| Later   | Strategic direction, not yet ready for near-term execution       |
| Parked  | Out of active sequence until evidence changes                    |

## Plan Update Workflow

Use `$dart-plan-update` in Codex or `/dart-plan-update` in Claude Code for
planning discussions and plan edits.
Use `$dart-next` or `/dart-next` when the goal is to select and execute the
next bounded task from the dashboard and other tracked evidence.

1. Load `docs/ai/north-star.md`, this file, `docs/plans/dashboard.md`, and
   `docs/plans/north-star-roadmap.md`.
2. Classify the request as discussion-only, plan edit, or task derivation.
3. Inspect current evidence before changing priority, timeline, scope, or
   status.
4. Propose the smallest plan change that keeps the roadmap coherent.
5. If editing, update the dashboard for operating state, the active numbered
   initiative file or external owner document for plan rationale, and any index
   links.
6. When deriving implementation packets or multi-session dev tasks, record the
   DART specification intake from `docs/ai/orchestration.md`: value, scope,
   non-goals, assumptions or open decisions, acceptance evidence, gates, and
   dependencies. Consequential ambiguity belongs in an owner-local
   `Decision needed` block.
7. Run the matching verification gates from `docs/ai/verification.md`.

Plan discussions can be exploratory. Plan edits should leave the files in a
state where the next agent can immediately tell what changed, why it changed,
and what to do next.

## Decision Needed Blocks

When consequential ambiguity blocks roadmap or owner-doc work, record the
decision locally instead of creating a global queue. Use this compact block in
the owning plan, design doc, or onboarding doc:

```markdown
> **Decision needed:** <question>
> **Choices:** <option A>; <option B>; <option C if needed>
> **Current default:** <what agents should assume until decided>
> **Evidence needed:** <repo evidence, maintainer input, CI/benchmark result>
> **Unblocks:** <plan packet, owner-doc update, or implementation path>
```

Keep these blocks for live ambiguity only. Once the decision is made, replace
the block with the decision and rationale in the owner doc, then update the
dashboard if plan state changes.

## Structural Checks

The living plan system relies on repository checks instead of manual memory:

- `pixi run check-ai-commands` verifies generated Codex adapter sync,
  workflow capability parity, workflow public paths, required-reading path
  existence, required-reading coverage in `docs/ai/workflows.md`, and
  approval-boundary wording.
- `pixi run check-docs-policy` verifies docs index coverage and active
  `docs/dev_tasks/<task>/` shape, including required `README.md` and
  `RESUME.md` files. It also bounds the plan dashboard: each `### PLAN-` entry
  must stay at or under 40 lines with a `- Next step:` field at or under 15
  lines, the dashboard must hold no `Status: Complete` entry (completed plans
  move to [`archive.md`](archive.md)), and every `### PLAN-` section in
  `archive.md` must record `**Final status:** Complete`.
- `pixi run sync-ai-commands` regenerates AI adapters from the `.claude/`
  workflow and skill sources before the non-mutating checks run.

Run the AI docs/adapters gate set from `docs/ai/verification.md` after changing
plan workflow sources, AI workflow docs, generated adapter sources, or active
dev-task shape rules.
