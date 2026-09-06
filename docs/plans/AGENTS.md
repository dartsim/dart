# docs/plans/

Agent rules for DART's living plans. `README.md` here owns the plan policy in
full; this file only lists what to check before editing.

- Start with `docs/ai/north-star.md`, then `README.md` here, then
  `dashboard.md`.
- `dashboard.md` is the single source of truth for priority, status, horizon,
  dimension, next step, and gate of operating plans. Numbered plan files own
  scope, workstreams, work packets, acceptance criteria, and a bounded
  progress log; they never repeat dashboard fields.
- Revise an existing initiative before adding one; keep initiative IDs stable.
- Completed plans move from `dashboard.md` to `archive.md` in the completing
  PR; their durable output moves to the owner selected by `docs/README.md`.
- Keep the directory flat: one `NNN-<initiative>.md` owner file plus an
  optional `NNN-<initiative>/` sidecar directory; never split by status,
  horizon, or dimension.

`pixi run check-docs-policy` enforces the dashboard budgets, plan and archive
shape, and plan-file discoverability; select the remaining gates with
`docs/ai/verification.md`.
