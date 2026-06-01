# North-Star Roadmap

This file explains the strategic shape of DART's path to the AI-native north
star. It does not own mutable tracking state.

For current priority, status, horizon, north-star dimension, and next step, use
[`dashboard.md`](dashboard.md). That dashboard is the single source of truth for
plan operating state.

## North-Star Dimensions

- **Easy start**: self-explanatory public APIs, package availability, Pixi
  source workflows, and cloud-runnable tutorials for users who need to try DART
  before local installation.
- **Algorithm extensibility**: stable research-facing extension points,
  baseline comparisons, benchmarks, and tests.
- **Scalable compute**: multi-core CPU, SIMD, and evidence-backed GPU planning.
- **AI-native execution**: agents can choose, execute, verify, and revise work
  from tracked evidence.
- **Release transition**: DART 7 clean-break hardening and DART 6.16
  compatibility support.

## Source-Of-Truth Split

| File                                         | Owns                                                                      |
| -------------------------------------------- | ------------------------------------------------------------------------- |
| [`dashboard.md`](dashboard.md)               | Plan priority, status, horizon, dimension, next step, and gate            |
| Detailed numbered initiative files           | Scope, workstreams, acceptance criteria, revision triggers, and rationale |
| External owner docs linked from dashboard    | Scope for initiatives that do not need a dedicated plan file yet          |
| `north-star-roadmap.md`                      | Strategic framing and sequencing principles                               |
| [`README.md`](README.md)                     | Planning rules and directory structure policy                             |
| [`../ai/north-star.md`](../ai/north-star.md) | Repository-level mission, current gaps, and readiness bar                 |

## Current Plan Set

Use [`dashboard.md`](dashboard.md) for the current plan catalog and ordering.
This file intentionally avoids repeating that table.

## Sequencing Principles

- Prioritize research readiness when choosing near-term work.
- Prefer evidence-backed plan revisions over speculative hierarchy changes.
- Turn multi-session implementation work into `docs/dev_tasks/`; keep durable
  strategic state in `docs/plans/`.
- Do not split plan files by status, horizon, or north-star dimension because
  those values move over time.
- Promote a plan into a subdirectory only when that individual initiative needs
  multiple durable planning artifacts.
