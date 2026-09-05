# DART Living Plans

`docs/plans/` turns the north star in [`../ai/north-star.md`](../ai/north-star.md)
into sequenced, evidence-backed work. Use it to decide what to do next, revise
priorities, or cut a strategic direction into bounded packets. Cross-bucket
placement rules live in [`../README.md`](../README.md).

For DART 7 readiness, start with
[PLAN-040](040-dart7-release-hardening.md): milestone scope, gaps, M1 dependency
map and acceptance. The dashboard still owns priority and initiative status;
packet state stays in each owning plan using the orchestration lifecycle.
Capability labels (`Implemented`, `Partial`, `Planned`, `Undecided`) describe
source/evidence coverage and are distinct from initiative/packet status.

## Files

Do not maintain a per-file inventory here; list the current set with
`ls docs/plans/` and read files by role:

| File                                                 | Owns                                                                                                          |
| ---------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| [`dashboard.md`](dashboard.md)                       | Operating view: priority (document order), status, horizon, dimension, next step, and gate of every open plan |
| [`archive.md`](archive.md)                           | Completed plans with their final outcome and closing evidence                                                 |
| `NNN-<initiative>.md`                                | One initiative: outcome, scope, workstreams, work packets, acceptance criteria, bounded progress log          |
| `NNN-<initiative>/`                                  | Optional sidecars for that initiative: audits, matrices, manifests, evidence packets                          |
| [`solver-family-intake.md`](solver-family-intake.md) | Intake checklist every new solver, algorithm, or paper implementation records first                           |
| [`AGENTS.md`](AGENTS.md)                             | Folder-local agent checklist                                                                                  |

External owner docs (a design doc, the release roadmap) may back a dashboard
entry when a dedicated plan file would only duplicate them.

## North-Star Dimensions

Each plan names the dimension it serves; the dashboard groups nothing by
dimension because plans move between them.

- **Easy start**: self-explanatory public APIs, package availability, Pixi
  source workflows, and cloud-runnable tutorials.
- **Algorithm extensibility**: stable research-facing extension points,
  baseline comparisons, benchmarks, and tests.
- **Scalable compute**: multi-core CPU, SIMD, and evidence-backed GPU work.
- **AI-native execution**: agents can choose, execute, verify, and revise work
  from tracked evidence.
- **Release transition**: DART 7 clean-break hardening and `release-6.*`
  compatibility support.

Sequencing principles: prioritize research readiness for near-term work;
prefer evidence-backed revisions over speculative hierarchy; keep multi-session
implementation in `docs/dev_tasks/` and durable strategy here.

## Lifecycle

Plans are living execution surfaces, not a knowledge base or an archive. A plan
helps choose the next task, then shrinks or disappears after its durable
output lands elsewhere.

1. Create or revise a numbered plan when a roadmap gap needs priority,
   sequencing, gates, and acceptance criteria.
2. Derive bounded work: a direct PR for small work, work packets (see
   [`../ai/orchestration.md`](../ai/orchestration.md)) for orchestrated work,
   or `docs/dev_tasks/<task>/` when implementation spans sessions.
3. Move durable output out as it lands: user instructions to
   `docs/readthedocs/` or `README.md`; developer knowledge to
   `docs/onboarding/`; architecture, API shape, and tradeoffs to
   `docs/design/`; release facts to the release and changelog owners.
4. When the outcome is achieved, move the dashboard entry to
   [`archive.md`](archive.md) in the same PR using the
   `**Final status:** Complete` shape, pointing at durable owner docs rather
   than the numbered plan file.
5. Delete or fold the numbered plan file once it no longer guides
   prioritization. Git history keeps old plan text.

Create a follow-up plan only when new roadmap work remains; do not append
"next phase" to the same initiative forever.

## Single Source Of Truth And Budgets

- `dashboard.md` owns priority, status, horizon, dimension, next step, and
  gate. Plan files and external owners point to it and never repeat those
  fields.
- Dashboard entries are git-history friendly: one plan per block, priority by
  document order, one frequently changed field per line.
- Each `### PLAN-` block stays at or under 40 lines and its `- Next step:` at
  or under 15 lines (enforced by `pixi run check-docs-policy`). Relocate
  overflow into the owner plan file's `## Progress log` and leave the current
  next action plus a `History:` pointer. Prune the progress log when its
  entries no longer inform the next step.
- Keep the directory flat. Add a `NNN-<initiative>/` sidecar directory only
  when one initiative needs several planning artifacts; keep the `.md` owner
  file in place and link every sidecar from it. Never split by status,
  horizon, or dimension.

## Initiative Card Shape

```markdown
# PLAN-000: Short Name

- Operating state: `PLAN-000` in [`dashboard.md`](dashboard.md)
- Outcome: <what success looks like>
- Current evidence: <repo docs, code, tests, CI, issue/PR state, benchmarks>
```

| Status   | Meaning                                            |
| -------- | -------------------------------------------------- |
| Proposed | Worth considering but not actively sequenced       |
| Active   | Currently part of the intended path                |
| Blocked  | Valuable but waiting on a named decision/evidence  |
| Complete | Outcome achieved; entry lives in `archive.md`      |
| Parked   | Intentionally deferred; revisit only with new data |

| Horizon | Meaning                                                          |
| ------- | ---------------------------------------------------------------- |
| Now     | Highest-priority work for the next planning/implementation cycle |
| Next    | Sequenced after current blockers or active tasks clear           |
| Later   | Strategic direction, not yet ready for near-term execution       |
| Parked  | Out of active sequence until evidence changes                    |

## Decision Needed Blocks

When consequential ambiguity blocks roadmap or owner-doc work, record it
locally in the owning plan, design, or handbook doc instead of a global queue:

```markdown
> **Decision needed:** <question>
> **Choices:** <option A>; <option B>; <option C if needed>
> **Current default:** <what agents should assume until decided>
> **Evidence needed:** <repo evidence, maintainer input, CI/benchmark result>
> **Unblocks:** <plan packet, owner-doc update, or implementation path>
```

Keep these blocks for live ambiguity only. Once decided, replace the block
with the decision and rationale, then update the dashboard if plan state
changed.

## Revising Plans

Use `/dart-plan-update` (Claude Code) or `$dart-plan-update` (Codex)
for plan discussions and edits, and `/dart-next` or `$dart-next` to select and
execute the next bounded task. Revise when the north star or a release
priority changes; when CI, benchmark, packaging, or downstream evidence changes
risk or sequencing; when a plan item is split, consolidated, completed,
blocked, or parked; when a dev task starts or completes; or when the dashboard
stops being a usable operating view.

Every plan edit cites repository docs, code, CI, issue/PR state, benchmark
data, or explicit maintainer direction, and leaves the files in a state where
the next agent can tell what changed, why, and what to do next. Derived
packets and dev tasks record the specification intake from
[`../ai/orchestration.md`](../ai/orchestration.md): value, scope, non-goals,
assumptions or open decisions, acceptance evidence, gates, and dependencies.

## Checks

`pixi run check-docs-policy` enforces dashboard budgets and field shape,
archive shape, plan-ID uniqueness, numbered-plan discoverability, and
sidecar reachability (blocking for `.md`, advisory for data files). `pixi run check-ai-commands` keeps the planning workflows'
required reading and generated adapters in sync. Select the full gate set with
[`../ai/verification.md`](../ai/verification.md).
