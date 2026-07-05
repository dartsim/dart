# Documentation Information Architecture

This file owns the conceptual structure of `docs/` on the DART 6.20 release
branch: what each documentation bucket is for, how agents should choose a home
for new knowledge, and when a branch-local folder split is justified.

## Design Goal

DART 6.20 documentation should help maintainers and agents answer four
release-branch questions without rediscovering the branch from raw code:

1. What guidance is public user documentation?
2. What guidance is durable contributor or maintainer workflow policy?
3. What state is temporary task handoff and must be retired?
4. Which command or gate proves the current claim?

The release branch optimizes for compatibility, low context cost, and stable
links. It deliberately keeps a smaller docs tree than `main`.

## Current Structure

| Conceptual bucket     | Current path        | Audience             | Lifecycle          | Owns                                                       |
| --------------------- | ------------------- | -------------------- | ------------------ | ---------------------------------------------------------- |
| Published user docs   | `docs/readthedocs/` | DART users           | durable, published | Public user docs, tutorials, and Read the Docs inputs      |
| Developer handbook    | `docs/onboarding/`  | maintainers, agents  | durable            | Build/test/CI/release/compatibility guidance for DART 6.20 |
| AI workflow policy    | `docs/ai/`          | agents, maintainers  | durable            | Release-branch AI principles, workflow map, sessions, and gates |
| Active task handoff   | `docs/dev_tasks/`   | agents, task owners  | temporary          | Multi-session implementation state and resume context      |
| API generation inputs | `docs/doxygen/`     | documentation tools  | durable input      | C++ API reference inputs                                   |

`docs/onboarding/` is the durable developer handbook path for this release
branch. Do not rename it during ordinary maintenance.

`main` carries a broader DART 7 docs structure with `docs/design/`,
`docs/plans/`, `docs/background/`, `docs/assets/`, and `docs/python_api/`.
Do not copy those buckets wholesale into `release-6.20`; add a new top-level
folder here only when the branch has a distinct release-maintenance lifecycle
that the existing buckets cannot represent.

## Placement Matrix

Use this matrix before creating or moving a release-branch doc:

| If the content is...                                             | Put it in...            | Do not put it in...                   |
| ---------------------------------------------------------------- | ----------------------- | ------------------------------------- |
| Public install, tutorial, or user concept guidance               | `docs/readthedocs/`     | `docs/onboarding/` as the only path   |
| Contributor workflow, build/test/CI/release operation            | `docs/onboarding/`      | `docs/ai/` unless the rule is AI-only |
| DART 6 compatibility or downstream maintenance policy            | `docs/onboarding/`      | `docs/dev_tasks/` after work lands    |
| AI agent rule, workflow map, session, or verification policy     | `docs/ai/`              | `docs/onboarding/` as source of truth |
| Multi-session implementation status, branch note, resume prompt  | `docs/dev_tasks/<task>/` | durable docs after task completion    |
| Generated or handwritten C++ API reference input                 | `docs/doxygen/`         | temporary task folders                |

## Dev-Task Retirement

`docs/dev_tasks/` owns branch/session-local handoff state only. Before a task
folder is deleted, move durable facts to the matching owner:

- public user guidance to `docs/readthedocs/`;
- maintainer, release, compatibility, or contributor guidance to
  `docs/onboarding/`;
- AI workflow or gate policy to `docs/ai/`;
- C++ API reference input to `docs/doxygen/`;
- source-level details to code comments when the code is the durable owner.

Do not leave important decisions only in a temporary task folder.

## Split And Rename Criteria

Add a new top-level docs folder on this release branch only when all of these
are true:

1. The content has a distinct release-maintenance lifecycle from every existing
   bucket.
2. At least three durable docs would move there immediately.
3. The split reduces required-reading ambiguity for agents.
4. `docs/README.md`, `docs/AGENTS.md`, relevant workflow sources, and generated
   adapters can be updated in the same PR.

Rename a top-level docs folder only through a dedicated compatibility migration
that updates links, workflows, generated adapters, and published references
together.

## Agent Routing Rules

When a prompt asks where release-branch documentation belongs:

1. Read this file, `docs/README.md`, and the local folder `README.md` or
   `AGENTS.md`.
2. Classify the content by lifecycle first, then audience, then topic.
3. Prefer updating an existing owner doc over creating a new file.
4. If a file is temporary task state, put it under `docs/dev_tasks/` and plan
   its deletion path up front.
5. If durable learning comes from task completion, promote it to
   `docs/onboarding/`, `docs/readthedocs/`, `docs/ai/`, `docs/doxygen/`, or code
   before deleting the task folder.

## Verification

For release-branch docs-only changes, run `pixi run lint`.

If the change touches AI workflow sources, generated adapters, or `docs/ai/`,
also run:

- `pixi run sync-ai-commands`
- `pixi run check-ai-commands`
