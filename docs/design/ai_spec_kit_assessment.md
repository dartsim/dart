# Spec Kit Assessment For DART AI Workflows

This design note records the DART-specific assessment of
[`github/spec-kit`](https://github.com/github/spec-kit) and the resulting
workflow decision. It is durable rationale, not active roadmap state.

## Evidence Reviewed

The assessment used a June 2026 snapshot of Spec Kit at commit `3e69233` and
inspected:

- `README.md` and `spec-driven.md`;
- command templates under `templates/commands/`;
- artifact templates under `templates/`;
- the Specify CLI integration and workflow code under `src/specify_cli/`;
- extension, preset, and workflow catalogs.

DART evidence came from `AGENTS.md`, `docs/ai/`, `docs/plans/`,
`docs/dev_tasks/`, `docs/onboarding/ai-tools.md`, `.claude/commands/`,
`.claude/skills/`, and `scripts/sync_ai_commands.py`.

## Spec Kit Model

Spec Kit is an AI project-infrastructure toolkit for spec-driven development.
Its core lifecycle is:

```text
constitution -> specify -> clarify -> checklist -> plan -> tasks -> analyze -> implement
```

The main working artifacts are per-feature specs, plans, research notes, data
models, contracts, quickstarts, task lists, and checklists. The strongest
ideas are:

- a constitution that makes project principles non-negotiable during planning;
- a strict separation between product intent (`what` and `why`) and technical
  plan (`how`);
- explicit ambiguity markers and targeted clarification before planning;
- checklists that act as "unit tests for English" by checking requirement
  clarity, completeness, consistency, and measurability;
- task generation with stable IDs, dependencies, exact file paths, optional
  parallel markers, and independently testable user-story slices;
- read-only cross-artifact analysis before implementation; and
- generated agent integrations from one workflow/template source.

## DART Model

DART already has a different AI-native substrate:

- `docs/ai/principles.md` is the constitution-equivalent policy surface.
- `docs/ai/workflows.md` and `docs/ai/capabilities.json` own workflow
  discoverability and machine-readable capability state.
- `.claude/commands/` and `.claude/skills/` are editable sources, while
  `.agents/skills/` and `.opencode/command/` are generated surfaces.
- `docs/plans/dashboard.md` owns operating state for living plans.
- Numbered plans and `docs/ai/orchestration.md` own work packets.
- `docs/dev_tasks/<task>/` owns active multi-session implementation state.
- `docs/ai/verification.md` maps work types to `pixi run ...` gates and
  objective-specific evidence.

DART's code is not routine generated output from feature specs. Physics-engine
changes need mathematical review, regression tests, API-boundary checks,
benchmark packets, determinism or replay evidence, downstream compatibility
judgment, and for solver/paper work the full paper-parity bar.

## Comparison

| Area                    | Spec Kit                                                 | DART fit                                                                                             |
| ----------------------- | -------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Governing rules         | `.specify/memory/constitution.md` drives plan gates      | Already covered by `docs/ai/principles.md`, `AGENTS.md`, and task-specific owner docs                |
| Feature artifacts       | `specs/<feature>/spec.md`, `plan.md`, `tasks.md`, etc.   | Active detail belongs in `docs/dev_tasks/<task>/`, numbered plans, or owner docs                     |
| Clarification           | Command-enforced targeted questions and markers          | Useful; should map to DART's owner-local `Decision needed` blocks                                    |
| Requirements checklists | Domain checklists for requirement quality                | Useful when expressed as acceptance evidence and gate selection, not as a second checklist tree      |
| Task execution          | User-story task lists, optional generated implementation | Partly useful; DART needs packet boundaries, exact gates, and empirical evidence                     |
| Generated agent files   | Specify CLI writes integrations for many tools           | DART already has `scripts/sync_ai_commands.py`; a second generator would duplicate ownership         |
| Extensions and hooks    | Extensible commands, presets, shell/workflow hooks       | High risk for DART unless reviewed and constrained; many hooks conflict with explicit-approval rules |

## Verdict

DART should use Spec Kit as inspiration, not install or adopt it directly as the
normal repository workflow.

Do not add `.specify/`, Spec Kit command files, or Spec Kit-managed feature
directories to the DART repository by default. Direct installation would create
a second workflow root beside `docs/ai/`, `docs/plans/`, and `docs/dev_tasks/`;
copy product-app-oriented templates into a C++ physics-engine context; duplicate
DART's existing generated adapter system; and introduce extension/hook behavior
that can conflict with DART's approval boundary for pushes, PR comments, CI
reruns, and other external mutations.

Using Spec Kit directly is reasonable only for an explicitly approved
throwaway experiment outside the tracked DART workflow, or after a maintainer
chooses to build and maintain a DART-specific Spec Kit preset/extension whose
outputs are reconciled with `docs/ai/`, `docs/plans/`, `docs/dev_tasks/`,
`pixi run ...` gates, and DART's generated adapter checks. Until that exists,
the portable value is the method, not the tool installation.

## Adopted DART Pattern

DART should fold the useful Spec Kit ideas into existing owner surfaces:

- Treat `docs/ai/principles.md` as the constitution and run its audit for
  substantial AI-infra work.
- Require multi-session or design-heavy tasks to record a compact
  specification intake: value, scope, non-goals, assumptions or open decisions,
  acceptance evidence, and gates.
- Use owner-local `Decision needed` blocks for consequential ambiguity instead
  of a global decision queue or silent assumptions.
- Require work packets to be executable only when objective, scope, non-goals,
  acceptance evidence, gates, and dependencies are concrete enough to verify.
- Keep active implementation detail in `docs/dev_tasks/<task>/` and promote
  durable decisions to `docs/design/`, `docs/plans/`, `docs/onboarding/`,
  `docs/readthedocs/`, code, tests, examples, or release docs before cleanup.
- Add automation only after the text pattern proves stable, following
  PLAN-121's advisory-first approach.
