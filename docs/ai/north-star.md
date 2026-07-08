# Release Branch North Star

For DART 6.20, the north star is a stable compatibility branch with a smaller
dependency footprint, preserved downstream Gazebo/gz-physics behavior, and
clear maintenance workflow support.

Near-term AI-assisted work should prioritize:

- one-dependency or one-vendored-tree cleanup PRs;
- compatibility evidence for package components and installed headers;
- release-branch CI and Gazebo gates;
- living roadmap state in `docs/plans/dashboard.md`;
- clean handoffs through `docs/dev_tasks/`;
- durable decisions promoted to `docs/design/`, `docs/onboarding/`,
  `docs/background/`, or `docs/readthedocs/` before task cleanup.

Do not use DART 7 clean-break assumptions as proof that a DART 6.20 removal is
safe. Use DART 7 only as reference evidence and then prove the release-branch
compatibility surface directly.

## Planning Surfaces

- `docs/plans/dashboard.md` owns current release-branch priority, status,
  horizon, next step, and gate.
- `docs/dev_tasks/` owns active multi-session task handoff.
- `docs/design/` owns durable technical and compatibility rationale.
- `docs/background/` owns reusable theory and reference foundations.
- `docs/onboarding/` owns landed maintainer and contributor workflow guidance.
- `docs/readthedocs/` owns published user guidance.
