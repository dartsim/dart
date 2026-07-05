# docs/

Agent entry point for DART documentation.

## Required Reading

Start with:

1. Root `AGENTS.md`
2. `docs/README.md`
3. `docs/information-architecture.md`
4. The local folder `README.md` or `AGENTS.md` for the surface you are editing

For AI-infra docs, also read `docs/ai/principles.md`,
`docs/ai/terminology.md`, `docs/ai/verification.md`, and
`docs/ai/components.md`.

## Bucket Map

| Path           | Use for                                                             |
| -------------- | ------------------------------------------------------------------- |
| `ai/`          | AI-native principles, terminology, workflows, sessions, and gates   |
| `assets/`      | Source-controlled docs assets outside RTD `_static/`                |
| `background/`  | Theory and research foundations                                     |
| `design/`      | Durable technical rationale and design decisions                    |
| `dev_tasks/`   | Temporary active multi-session task tracking                        |
| `doxygen/`     | C++ API Doxygen inputs                                              |
| `onboarding/`  | Current developer handbook path: build/test/CI/release/API guidance |
| `plans/`       | Living roadmap, priority, dependencies, gates, and plan lifecycle   |
| `python_api/`  | Reusable dartpy API module pages                                    |
| `readthedocs/` | Published user documentation site source                            |

## Placement Rules

- Classify docs by lifecycle first, then audience, then topic.
- Prefer updating an existing owner doc over adding a new file.
- Put mutable priority, sequencing, next-step, and gate state in `plans/`, not
  `design/`.
- Put active handoff and resume state in `dev_tasks/`; delete the folder when
  the task completes after durable content is promoted.
- Put user-facing documentation in `readthedocs/` or root user entrypoints.
- Keep `onboarding/` as the current handbook path unless a dedicated migration
  updates all links, workflows, generated adapters, and policy checks together.

See `docs/information-architecture.md` for the full placement matrix and future
split or rename criteria.

## Verification

Use `docs/ai/verification.md` to select gates. For docs-structure or placement
policy changes, run at least:

- `pixi run lint-md`
- `pixi run check-lint-md`
- `pixi run check-docs-policy`
- `pixi run check-lint-spell`
