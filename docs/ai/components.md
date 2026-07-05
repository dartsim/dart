---
type: ai-component-policy
owner: self
---

# AI Components

This document defines how DART maintains AI-facing components.

## Ownership Model

`AGENTS.md` is the root pointer board. `docs/ai/` owns durable AI-native policy.
`docs/ai/terminology.md` owns canonical AI-facing terms.
`docs/onboarding/ai-tools.md` owns compatibility details. The current editable
workflow source is `.claude/commands/`, and the current editable domain-skill
source is `.claude/skills/`.
`docs/ai/capabilities.json` owns machine-readable capability status, category,
and gate profile; `docs/ai/workflows.md` owns the human-readable public paths
and gate details.

The Markdown files directly under `docs/ai/` use a narrow frontmatter pilot
with only stable identity fields: `type` and `owner`. The pilot does not carry
mutable status, freshness, or relationship lists; those stay in the owner docs,
the plan dashboard, or generated manifests with an explicit consumer.

Generated adapter entrypoints are first-class entrypoints for their tools:

- `.codex/skills/` for Codex;
- `.opencode/command/` for OpenCode.

Generated files include source metadata and must not be hand-edited.

## Adding A Workflow

1. Add the concise workflow source under `.claude/commands/dart-<name>.md`.
2. Keep detailed policy in `docs/ai/` or `docs/onboarding/`.
3. Add the workflow to `docs/ai/workflows.md`.
4. Add the workflow to `docs/ai/capabilities.json`.
5. Run `pixi run sync-ai-commands`.
6. Run `pixi run check-ai-commands`.

## Adding A Domain Skill

1. Add `.claude/skills/dart-<name>/SKILL.md`.
2. Keep the skill lightweight and point to full docs.
3. Add the skill to `docs/ai/workflows.md`.
4. Add the skill to `docs/ai/capabilities.json`.
5. Run `pixi run sync-ai-commands`.
6. Run `pixi run check-ai-commands`.

## Improving AI Infra From Learnings

Use `dart-retrospect` after a completed session only when the learning is
general enough to help future agents or contributors. Skip routine work,
one-off local choices, and review-only narrative.

Retrospective docs edits are local work when the user requested them. GitHub,
CI, branch, and review-thread mutations still require explicit approval.

Route durable learnings to one owner:

- Cross-session axioms and audit questions: `docs/ai/principles.md`, kept
  compact.
- Canonical AI-facing terms and migration candidates:
  `docs/ai/terminology.md`.
- Gate selection and evidence expectations: `docs/ai/verification.md`.
- Workflow routing and public paths: `docs/ai/workflows.md`.
- Machine-readable capability status, category, and gate profile:
  `docs/ai/capabilities.json`.
- AI component mechanics, source surfaces, and structural checks: this file.
- Tool compatibility and generated-adapter caveats:
  `docs/onboarding/ai-tools.md`.
- Documentation bucket placement and future restructure criteria:
  `docs/information-architecture.md`.
- Reusable user-invoked workflow: `.claude/commands/`, synced to generated
  adapters.
- Reusable on-demand domain knowledge: `.claude/skills/`, synced to generated
  adapters.
- Future agents, hooks, or scripts: add only when the responsibility or
  lifecycle trigger is stable enough to justify a maintained component.
- Project, problem, component, or plan-specific learning: the relevant
  `docs/` owner, such as `docs/plans/`, `docs/onboarding/`, or a module
  `AGENTS.md`.
- Plan lifecycle policy: `docs/plans/README.md` and `docs/plans/AGENTS.md`.
  Plans should drive current work and then move durable output to
  `docs/readthedocs/`, `docs/onboarding/`, `docs/design/`, code, tests,
  examples, or release docs.

Prefer improving or consolidating an existing component over adding a new
surface. If two docs would need the same changing fact, pick one owner and make
the other a pointer.

Durable retrospect learnings must be discoverable from the owner surface they
improve. Prefer editing an existing owner doc. If a learning justifies a new
durable file, link that file from the relevant owner index or plan before the
temporary session or dev-task context is retired.

## Public Path Requirement

Every AI workflow must map back to public docs and `pixi run ...` commands so a
contributor can complete the same work manually. AI tooling can make the path
faster; it must not be the only path.

## Checks

`scripts/sync_ai_commands.py` is the implementation behind
`pixi run sync-ai-commands` and `pixi run check-ai-commands`.

`pixi run sync-ai-commands` updates generated `.codex/skills/` and
`.opencode/command/` files from `.claude/` sources.

`pixi run check-ai-commands` is the non-mutating CI check. It verifies:

- generated adapter sync;
- machine-readable capability manifest coverage;
- effective capability parity across Claude Code, OpenCode, and Codex;
- command and skill frontmatter, descriptions, and size budgets;
- required command structure: an `argument-hint` frontmatter key plus
  `## Required Reading`, `## Workflow`, and `## Output` sections in order;
- required `docs/ai/` policy documents exist;
- `AGENTS.md` and `docs/README.md` point to the AI entrypoint;
- `AGENTS.md` points to the workflow catalog and generated surfaces;
- `docs/ai/workflows.md` capability rows;
- public path and gate evidence for each workflow;
- required-reading entries are represented in each workflow row;
- `dart-docs-update` always loads `docs/AGENTS.md` and
  `docs/information-architecture.md` so documentation edits cannot bypass the
  docs placement owner;
- approval-boundary wording around GitHub, CI, branch, and review-thread
  mutations;
- private-path references in `docs/ai/`.

`pixi run check-docs-policy` also enforces documentation lifecycle rules that
are outside generated-adapter sync, including docs bucket visibility,
the root information-architecture owner links, dev-task shape, plan cleanup
invariants, dashboard entry budgets (at most 40 lines per `### PLAN-` block
and 15 lines per `- Next step:` field), the no-`Complete`-entries rule for
`docs/plans/dashboard.md` (completed plans move to `docs/plans/archive.md`,
whose entry shape is also checked), the `docs/ai/` frontmatter pilot, and
the `docs/readthedocs/papers.md` catalog schema, pilot-scoped internal
Markdown links, and conservative owner-index discoverability (both promoted
from advisory to blocking after the PLAN-121 pilot inventory reported a clean
cycle). North-star evidence freshness remains a report-only advisory by
design: blocking on git-date freshness would fail unrelated PRs whenever
cited evidence paths change, so clearing that channel stays a manual,
owner-doc-governed re-verification step.

Report-only checks still need a quiet default run. Before adding a new advisory
to a mandatory command, scope it to an owned pilot surface, clean the existing
inventory, or add an explicit baseline so routine verification output remains
actionable. A report-only signal that always prints legacy debt is not ready to
be a shared gate.

These checks are structural. The principle audit in `docs/ai/principles.md`
owns judgment calls such as source-of-truth placement, public path quality, and
whether a change is simpler than the alternatives. `docs/ai/verification.md`
owns gate selection and evidence mapping.
