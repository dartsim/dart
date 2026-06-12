# DART Living Plans

This directory holds DART's living plan from current state to the AI-native
north star. Use it when deciding what to do next, revising priorities, or
turning a strategic direction into bounded work.

## Why `docs/plans/`

`docs/plans/` is a collection of living planning docs, so it follows the
plural collection style used by `docs/dev_tasks/`. It is separate from
`docs/dev_tasks/`:

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
4. Mark the dashboard entry `Complete` only after the durable owner docs,
   examples, tests, or code hold the result. Completed dashboard entries should
   point to those durable owner docs, not to an archival numbered plan file.
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

| File                                                                                                                                                                 | Purpose                                                                  |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| [`README.md`](README.md)                                                                                                                                             | Planning rules, structure, and revision workflow                         |
| [`dashboard.md`](dashboard.md)                                                                                                                                       | Single source of truth for plan operating state                          |
| [`north-star-roadmap.md`](north-star-roadmap.md)                                                                                                                     | Strategic framing and sequencing principles                              |
| [`solver-family-intake.md`](solver-family-intake.md)                                                                                                                 | PLAN-020 solver-family intake checklist for paper/solver work            |
| [`030-compute-resource-access/`](030-compute-resource-access/)                                                                                                       | PLAN-030 resource-access mission and evaluator contract                  |
| [`035-native-collision-dashboard.md`](035-native-collision-dashboard.md)                                                                                             | Durable native-collision feature/performance dashboard                   |
| [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)                                                                                 | Durable row-level native-collision coverage matrix sidecar               |
| [`035-native-collision/benchmark-manifest.md`](035-native-collision/benchmark-manifest.md)                                                                           | Generated native-collision benchmark evidence manifest                   |
| [`041-official-simulation-api-promotion.md`](041-official-simulation-api-promotion.md)                                                                               | PLAN-041 official DART 7 simulation API promotion sequence               |
| [`042-dart7-public-api-and-source-layout.md`](042-dart7-public-api-and-source-layout.md)                                                                             | PLAN-042 DART 7 public API topology and source-layout renovation         |
| [`042-dart7-public-api-and-source-layout/api-source-layout-audit.md`](042-dart7-public-api-and-source-layout/api-source-layout-audit.md)                             | PLAN-042 initial API/source-layout decision and audit packet             |
| [`042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md`](042-dart7-public-api-and-source-layout/post-promotion-source-layout-decision.md) | PLAN-042 post-promotion folder decision and follow-up gates              |
| [`081-deformable-implicit-barrier-solver.md`](081-deformable-implicit-barrier-solver.md)                                                                             | PLAN-081 deformable IPC scope, slices, and acceptance evidence           |
| [`081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md`](081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md)                                     | PLAN-081 IPC paper/repository parity checklist                           |
| [`081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md)                                   | PLAN-081 PD-IPC GPU source audit and implementation sequence             |
| [`081-deformable-implicit-barrier-solver/spb-gap-audit.md`](081-deformable-implicit-barrier-solver/spb-gap-audit.md)                                                 | PLAN-081 SPB source audit and self-intersection recovery plan            |
| [`082-rigid-implicit-barrier-contact.md`](082-rigid-implicit-barrier-contact.md)                                                                                     | PLAN-082 rigid IPC scope, manifest, and acceptance criteria              |
| [`082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md`](082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md)                               | PLAN-082 simultaneous-impact intake and go/no-go sidecar                 |
| [`083-unified-newton-barrier-multibody.md`](083-unified-newton-barrier-multibody.md)                                                                                 | PLAN-083 unified IPC/ABD Newton-barrier multibody direction              |
| [`083-unified-newton-barrier-multibody/ipc-variant-consolidation.md`](083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)                             | PLAN-083 IPC-family variant ownership and API consolidation map          |
| [`083-unified-newton-barrier-multibody/ppf-contact-solver-intake.md`](083-unified-newton-barrier-multibody/ppf-contact-solver-intake.md)                             | PLAN-083 PPF cubic-barrier, API, GPU, and platform intake                |
| [`083-unified-newton-barrier-multibody/implementation-roadmap.md`](083-unified-newton-barrier-multibody/implementation-roadmap.md)                                   | PLAN-083 phased CPU/GPU implementation and completion roadmap            |
| [`083-unified-newton-barrier-multibody/paper-deck-manifest.md`](083-unified-newton-barrier-multibody/paper-deck-manifest.md)                                         | PLAN-083 unified paper/deck figure, unit, benchmark, and demo manifest   |
| [`083-unified-newton-barrier-multibody/shared-primitive-audit.md`](083-unified-newton-barrier-multibody/shared-primitive-audit.md)                                   | PLAN-083 current-state shared primitive promotion audit                  |
| [`083-unified-newton-barrier-multibody/primitive-promotion-slice.md`](083-unified-newton-barrier-multibody/primitive-promotion-slice.md)                             | PLAN-083 first shared primitive implementation design                    |
| [`083-unified-newton-barrier-multibody/abd-first-slice-design.md`](083-unified-newton-barrier-multibody/abd-first-slice-design.md)                                   | PLAN-083 first affine-body dynamics implementation design                |
| [`091-architecture-hardening.md`](091-architecture-hardening.md)                                                                                                     | PLAN-091 architecture-hardening work packets from the 2026-06 assessment |
| [`101-dartsim-gui-simulator.md`](101-dartsim-gui-simulator.md)                                                                                                       | PLAN-101 dartsim GUI simulator scope, phases, and acceptance             |
| [`103-examples-strategy.md`](103-examples-strategy.md)                                                                                                               | PLAN-103 Python-first examples strategy                                  |
| [`103-examples-strategy/rigid-body-visual-verification.md`](103-examples-strategy/rigid-body-visual-verification.md)                                                 | PLAN-103 rigid-body py-demo visual evidence packet                       |
| [`104-vertex-block-descent-solver.md`](104-vertex-block-descent-solver.md)                                                                                           | PLAN-104 VBD solver scope, remaining corpus/performance work             |
| [`104-vertex-block-descent-solver/vbd-paper-gap-audit.md`](104-vertex-block-descent-solver/vbd-paper-gap-audit.md)                                                   | PLAN-104 VBD paper/reference gap audit                                   |
| [`104-vertex-block-descent-solver/ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md)                                                               | PLAN-104 OGC source audit and implementation sequence                    |
| [`104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](104-vertex-block-descent-solver/avbd-paper-gap-audit.md)                                                 | PLAN-104 AVBD paper/reference gap audit                                  |
| [`105-simplicits-geometry-agnostic-elastic-solver.md`](105-simplicits-geometry-agnostic-elastic-solver.md)                                                           | PLAN-105 Simplicits geometry-agnostic reduced elastic solver scope       |
| [`110-differentiable-simulation.md`](110-differentiable-simulation.md)                                                                                               | PLAN-110 differentiable simulation scope, audits, and gates              |
| [`120-inverse-kinematics-and-motion.md`](120-inverse-kinematics-and-motion.md)                                                                                       | PLAN-120 IK solver portfolio, whole-body parity, and motion IK           |
| Numbered initiative files, such as `010-*.md`                                                                                                                        | Active plan scope, evidence, open gaps, and acceptance criteria          |
| [`AGENTS.md`](AGENTS.md)                                                                                                                                             | Local rules for agents editing plan docs                                 |
| [`../ai/north-star.md`](../ai/north-star.md)                                                                                                                         | Mission, current state, missing capabilities, and readiness bar          |
| [`../ai/verification.md`](../ai/verification.md)                                                                                                                     | Completion audit and gate-selection rules                                |
| External owner docs linked from `dashboard.md`                                                                                                                       | Source docs for initiatives that do not need a plan file yet             |

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
- the dashboard no longer fits on one screen or stops being useful as an
  operating view.
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

Use `$dart-plan-update` in Codex or `/dart-plan-update` in Claude/OpenCode for
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
6. Run the matching verification gates from `docs/ai/verification.md`.

Plan discussions can be exploratory. Plan edits should leave the files in a
state where the next agent can immediately tell what changed, why it changed,
and what to do next.

## Structural Checks

The living plan system relies on repository checks instead of manual memory:

- `pixi run check-ai-commands` verifies generated Codex/OpenCode adapter sync,
  workflow capability parity, workflow public paths, required-reading path
  existence, required-reading coverage in `docs/ai/workflows.md`, and
  approval-boundary wording.
- `pixi run check-docs-policy` verifies docs index coverage and active
  `docs/dev_tasks/<task>/` shape, including required `README.md` and
  `RESUME.md` files.
- `pixi run sync-ai-commands` regenerates AI adapters from the `.claude/`
  workflow and skill sources before the non-mutating checks run.

Run the AI docs/adapters gate set from `docs/ai/verification.md` after changing
plan workflow sources, AI workflow docs, generated adapter sources, or active
dev-task shape rules.
