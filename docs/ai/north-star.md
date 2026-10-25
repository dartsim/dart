# Development Branch North Star

For `main`, the north star is the Python-first DART 7 program
([PLAN-624](../plans/dashboard.md)): a modern, typed dartpy over a core that
keeps the current solver as its baseline. The core gains multi-physics,
batching, and replay readiness behind bit-identical gates. It also keeps the
C++ API that Gazebo/gz-physics and gz-sim use compatible. `release-6.20`
keeps the DART 6
north star of a stable maintenance line.

Near-term AI-assisted work should prioritize:

- PLAN-624 milestone packets, in the program's order and behind its gates;
- one-dependency or one-vendored-tree cleanup PRs;
- compatibility evidence for package components and installed headers;
- development-branch CI and Gazebo gates;
- branch-local model/tool audits that keep durable project context and
  text-first, semantically inspected Filament verification discoverable as agents
  evolve;
- living roadmap state in `docs/plans/dashboard.md`;
- clean handoffs through `docs/dev_tasks/`;
- durable decisions promoted to `docs/design/`, `docs/onboarding/`,
  `docs/background/`, or `docs/readthedocs/` before task cleanup.

Prove the Gazebo-facing compatibility surface directly before treating a
removal as safe.

## Planning Surfaces

- `docs/plans/dashboard.md` owns current development-branch priority, status,
  horizon, next step, and gate.
- `docs/dev_tasks/` owns active multi-session task handoff.
- `docs/design/` owns durable technical and compatibility rationale.
- `docs/background/` owns reusable theory and reference foundations.
- `docs/onboarding/` owns landed maintainer and contributor workflow guidance.
- `docs/readthedocs/` owns published user guidance.
