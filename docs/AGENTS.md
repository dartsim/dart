# docs/

Agent entry point for DART 6.20 documentation.

## Required Reading

Start with:

1. Root `AGENTS.md`
2. `docs/README.md`
3. `docs/information-architecture.md`
4. The local folder `README.md` or `AGENTS.md` for the surface you are editing

For AI workflow docs, also read `docs/ai/principles.md`,
`docs/ai/verification.md`, and `docs/ai/components.md`.

## Bucket Map

| Path           | Use for                                                        |
| -------------- | -------------------------------------------------------------- |
| `ai/`          | Release-branch AI workflow policy, gates, sessions, and map    |
| `dev_tasks/`   | Temporary active multi-session task tracking                   |
| `doxygen/`     | C++ API Doxygen inputs                                         |
| `onboarding/`  | Durable DART 6.20 contributor, maintainer, build/test guidance |
| `readthedocs/` | Published user documentation source                            |

## Placement Rules

- Classify docs by lifecycle first, then audience, then topic.
- Prefer updating an existing owner doc over adding a new file.
- Put active handoff and resume state in `dev_tasks/`; delete the folder when
  the task completes after durable content is promoted.
- Put contributor, maintainer, build/test, release, compatibility, and
  branch-operation guidance in `onboarding/`.
- Put published user documentation in `readthedocs/`; keep API-generation input
  in `doxygen/`.
- Do not add DART 7-only buckets such as `docs/design/`, `docs/plans/`,
  `docs/background/`, or `docs/assets/` to this release branch unless a
  dedicated release-maintenance PR justifies the lifecycle split.

See `docs/information-architecture.md` for the full release-branch placement
matrix.
