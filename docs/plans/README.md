# DART 6 Living Plans

This directory holds DART 6.20's living plan state from the current release
branch toward the DART 6 AI-native maintenance north star. Use it when deciding
what to do next, revising priorities, or turning release-branch direction into
bounded work.

## Why `docs/plans/`

`docs/plans/` is separate from `docs/dev_tasks/`.

- `docs/plans/` records current priority, open gaps, sequencing, gates, and
  acceptance criteria.
- `docs/design/` records durable design proposals and decision rationale.
- `docs/dev_tasks/` tracks active multi-session implementation work and is
  deleted when that work completes.
- `docs/onboarding/` keeps landed contributor and maintainer guidance.

For cross-bucket placement decisions, use
[`../information-architecture.md`](../information-architecture.md).

## Files

| File | Purpose |
| --- | --- |
| [`dashboard.md`](dashboard.md) | Single source of truth for operating DART 6.20 plan state. |
| [`north-star-roadmap.md`](north-star-roadmap.md) | Release-branch sequencing principles and planning constraints. |
| [`archive.md`](archive.md) | Completed plan entries after their durable output moves elsewhere. |
| [`AGENTS.md`](AGENTS.md) | Local rules for agents editing plan docs. |

## Lifecycle

1. Create or update a plan when a roadmap gap needs priority, sequencing,
   gates, and acceptance criteria.
2. Derive bounded work from the plan. Use a direct PR for small work, or
   `docs/dev_tasks/<task>/` when implementation needs multi-session tracking.
3. Move durable output out of the plan when the work lands:
   - user instructions to `docs/readthedocs/` or `README.md`;
   - contributor and maintainer guidance to `docs/onboarding/`;
   - durable architecture and compatibility decisions to `docs/design/`;
   - theory and paper/reference context to `docs/background/`;
   - reusable media to `docs/assets/`.
4. When a plan completes, move its entry from `dashboard.md` to
   [`archive.md`](archive.md) in the same PR and point to the durable owner.

## Dashboard Shape

Each `dashboard.md` entry uses this shape:

```markdown
### PLAN-620: Short Name

- Owner doc: [path](path)
- Status: Active
- Horizon: Now
- Dimension: Compatibility and release maintenance
- Next step: <current next action>
- Gate: `pixi run lint`; <objective-specific gate>
```

Status values are `Proposed`, `Active`, `Blocked`, `Complete`, and `Parked`.
Horizon values are `Now`, `Next`, `Later`, and `Parked`.

Keep the dashboard as an operating view. Detailed evidence, packet boards, and
historical narrative belong in the linked owner doc.
