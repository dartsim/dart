# docs/

Agent entry point for DART 6.20 documentation.

## Required Reading

Start with:

1. Root `AGENTS.md`
2. `docs/README.md`
3. `docs/information-architecture.md`
4. The local folder `README.md` or `AGENTS.md` for the surface you are editing

For AI workflow docs, also read `docs/ai/principles.md`,
`docs/ai/terminology.md`, `docs/ai/orchestration.md`,
`docs/ai/verification.md`, and `docs/ai/components.md`.

## Bucket Map

| Path           | Use for                                                                 |
| -------------- | ----------------------------------------------------------------------- |
| `ai/`          | Release-branch AI workflow policy, terminology, gates, sessions, map    |
| `assets/`      | Durable repository documentation assets outside RTD `_static/`          |
| `background/`  | Theory, paper, and reference foundations                                |
| `design/`      | Durable DART 6.20 technical decisions and compatibility rationale       |
| `dev_tasks/`   | Temporary active multi-session task tracking                            |
| `doxygen/`     | C++ API Doxygen inputs                                                  |
| `onboarding/`  | Durable DART 6.20 contributor, maintainer, build/test guidance          |
| `plans/`       | Living release-branch roadmap state, priority, gates, and sequencing    |
| `readthedocs/` | Published user documentation source                                     |

## Placement Rules

- Classify docs by lifecycle first, then audience, then topic.
- Prefer updating an existing owner doc over adding a new file.
- Put mutable priority, sequencing, next-step, and gate state in `plans/`.
- Put durable technical decisions, architecture tradeoffs, and compatibility
  rationale in `design/`.
- Put theory, paper, and reusable reference foundations in `background/`.
- Put active handoff and resume state in `dev_tasks/`; delete the folder when
  the task completes after durable content is promoted.
- Put contributor, maintainer, build/test, release, compatibility, and
  branch-operation guidance in `onboarding/`.
- Put published user documentation in `readthedocs/`; keep API-generation input
  in `doxygen/`.
- Put durable reusable docs media in `assets/`; put RTD-only static assets in
  `readthedocs/_static/`.
- Do not import DART 7-only plan/design content wholesale. Adapt each bucket to
  the DART 6 compatibility lane and prove release-branch claims directly.

See `docs/information-architecture.md` for the full release-branch placement
matrix.
