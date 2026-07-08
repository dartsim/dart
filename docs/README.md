# DART Documentation

This release branch keeps public user documentation under `docs/readthedocs/`
and agent/developer workflow guidance under the folders below. DART 6.20 is an
AI-native maintenance lane, so it has the lifecycle buckets needed for plans,
durable decisions, theory references, and reusable documentation assets while
preserving the release branch's smaller compatibility-first scope.

For placement rules, lifecycle boundaries, and release-branch restructure
criteria, see
[`information-architecture.md`](information-architecture.md).

- [`ai/README.md`](ai/README.md): AI workflow entrypoint for DART 6.20 work.
- [`assets/`](assets/): durable repository documentation assets outside the
  Read the Docs `_static/` tree.
- [`background/`](background/README.md): theory, paper, and reference
  foundations.
- [`design/`](design/README.md): durable DART 6.20 technical decisions and
  compatibility rationale.
- [`dev_tasks/README.md`](dev_tasks/README.md): temporary multi-session task
  tracking.
- [`doxygen/`](doxygen/): Doxygen inputs consumed by C++ API docs.
- [`onboarding/`](onboarding/): build, test, CI, contribution, and release
  workflow references for maintainers and agents.
- [`plans/`](plans/README.md): living release-branch roadmap state, priority,
  gates, and sequencing.
- [`readthedocs/`](readthedocs/): public user documentation source.
