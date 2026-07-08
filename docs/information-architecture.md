# Documentation Information Architecture

This file owns the conceptual structure of `docs/` on the DART 6.20 release
branch: what each documentation bucket is for, how agents should choose a home
for new knowledge, and when a branch-local folder split is justified.

## Design Goal

DART 6.20 documentation should help maintainers and agents answer six
release-branch questions without rediscovering the branch from raw code:

1. What guidance is public user documentation?
2. What guidance is durable contributor or maintainer workflow policy?
3. What priority or gate state is mutable roadmap state?
4. What decision rationale should survive task cleanup?
5. What theory or reference context should be reused across tasks?
6. Which command or gate proves the current claim?

The release branch optimizes for compatibility, low context cost, and stable
links. It deliberately keeps a smaller docs tree than `main`, but it now has
the lifecycle buckets required for AI-native maintenance work.

## Current Structure

| Conceptual bucket     | Current path        | Audience             | Lifecycle           | Owns                                                       |
| --------------------- | ------------------- | -------------------- | ------------------- | ---------------------------------------------------------- |
| Published user docs   | `docs/readthedocs/` | DART users           | durable, published  | Public user docs, tutorials, and Read the Docs inputs      |
| Developer handbook    | `docs/onboarding/`  | maintainers, agents  | durable             | Build/test/CI/release/compatibility guidance for DART 6.20 |
| AI workflow policy    | `docs/ai/`          | agents, maintainers  | durable             | Release-branch AI principles, terminology, workflow map, sessions, orchestration, and gates |
| Living roadmap        | `docs/plans/`       | maintainers, agents  | living              | Priority, horizon, active gaps, next steps, gates, and acceptance criteria |
| Design rationale      | `docs/design/`      | maintainers, agents  | durable, revisable  | Architecture, API shape, compatibility constraints, tradeoffs, and accepted decisions |
| Theory background     | `docs/background/`  | maintainers, agents  | durable, reference  | Physics, math, solver, paper, and research foundations     |
| Active task handoff   | `docs/dev_tasks/`   | agents, task owners  | temporary           | Multi-session implementation state and resume context      |
| API generation inputs | `docs/doxygen/`     | documentation tools  | durable input       | C++ API reference inputs                                   |
| Shared assets         | `docs/assets/`      | docs authors         | durable asset input | Source-controlled docs assets outside site-local `_static/` |

`docs/onboarding/` is the durable developer handbook path for this release
branch. Do not rename it during ordinary maintenance.

`main` carries a broader DART 7 docs structure. DART 6.20 now adopts the
AI-infrastructure buckets that solve release-branch lifecycle problems, but it
does not copy DART 7's contents wholesale. Clean-break DART 7 architecture,
GPU/public API plans, and DART 7-only docs-build structure stay on `main`
unless a dedicated release-maintenance PR justifies a scoped backport.

`docs/python_api/` is intentionally not part of this DART 6 AI-infra split.
The release branch still owns dartpy user docs under `docs/readthedocs/dartpy/`
and C++ API generation input under `docs/doxygen/`.

## Placement Matrix

Use this matrix before creating or moving a release-branch doc:

| If the content is...                                             | Put it in...             | Do not put it in...                   |
| ---------------------------------------------------------------- | ------------------------ | ------------------------------------- |
| Public install, tutorial, or user concept guidance               | `docs/readthedocs/`      | `docs/onboarding/` as the only path   |
| Contributor workflow, build/test/CI/release operation            | `docs/onboarding/`       | `docs/ai/` unless the rule is AI-only |
| DART 6 compatibility or downstream maintenance policy            | `docs/onboarding/` or `docs/design/` by lifecycle | `docs/dev_tasks/` after work lands |
| AI agent rule, terminology, workflow map, session, orchestration, or verification policy | `docs/ai/` | `docs/onboarding/` as source of truth |
| Priority, horizon, next step, gate, active gap, or plan state    | `docs/plans/`            | `docs/design/` or `docs/onboarding/`  |
| Durable architecture/API tradeoff, accepted decision, or compatibility rationale | `docs/design/` | `docs/plans/` for mutable operating fields |
| Theory, paper, math, solver, or research reference foundation    | `docs/background/`       | `docs/design/` unless it is a DART decision |
| Multi-session implementation status, branch note, resume prompt  | `docs/dev_tasks/<task>/` | durable docs after task completion    |
| Generated or handwritten C++ API reference input                 | `docs/doxygen/`          | temporary task folders                |
| Durable docs image, diagram, or small media asset outside RTD static files | `docs/assets/` | task folders unless temporary |

## Dev-Task Retirement

`docs/dev_tasks/` owns branch/session-local handoff state only. Before a task
folder is deleted, move durable facts to the matching owner:

- public user guidance to `docs/readthedocs/`;
- maintainer, release, compatibility, or contributor guidance to
  `docs/onboarding/`;
- AI workflow or gate policy to `docs/ai/`;
- roadmap priority, open gaps, next steps, or gates to `docs/plans/`;
- durable technical decisions and compatibility rationale to `docs/design/`;
- theory, paper, or reference foundations to `docs/background/`;
- reusable documentation media to `docs/assets/`;
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
   `docs/onboarding/`, `docs/design/`, `docs/plans/`, `docs/background/`,
   `docs/readthedocs/`, `docs/ai/`, `docs/assets/`, `docs/doxygen/`, or code
   before deleting the task folder.

## Verification

For release-branch docs-only changes, run `pixi run lint`.

If the change touches AI workflow sources, generated adapters, or `docs/ai/`,
also run:

- `pixi run sync-ai-commands`
- `pixi run check-ai-commands`
