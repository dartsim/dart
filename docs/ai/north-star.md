# Development Branch North Star

For DART 6.20, the north star is a stable development branch with a smaller
dependency footprint, preserved downstream Gazebo/gz-physics behavior, and
clear maintenance workflow support.

Near-term AI-assisted work should prioritize:

- one-dependency or one-vendored-tree cleanup PRs;
- compatibility evidence for package components and installed headers;
- development-branch CI and Gazebo gates;
- branch-local model/tool audits that keep durable project context and
  text-first, semantically inspected OSG verification discoverable as agents
  evolve;
- living roadmap state in `docs/plans/dashboard.md`;
- clean handoffs through `docs/dev_tasks/`;
- durable decisions promoted to `docs/design/`, `docs/onboarding/`,
  `docs/background/`, or `docs/readthedocs/` before task cleanup.

Prove the DART 6 compatibility surface directly before treating a DART
6.20 removal as safe.

## Planning Surfaces

- `docs/plans/dashboard.md` owns current development-branch priority, status,
  horizon, next step, and gate.
- `docs/dev_tasks/` owns active multi-session task handoff.
- `docs/design/` owns durable technical and compatibility rationale.
- `docs/background/` owns reusable theory and reference foundations.
- `docs/onboarding/` owns landed maintainer and contributor workflow guidance.
- `docs/readthedocs/` owns published user guidance.
