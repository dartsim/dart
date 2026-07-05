# Documentation Information Architecture

This file owns the conceptual structure of `docs/`: what each documentation
bucket is for, how agents should choose a home for new knowledge, and when a
folder split or rename is justified.

## Design Goal

DART documentation should let a fresh human or AI agent answer four questions
without rediscovering the project from raw code:

1. What is DART trying to become?
2. What is the current state and next work?
3. Where does durable knowledge belong after a task lands?
4. Which command or gate proves the current claim?

The structure optimizes for source-of-truth ownership, low context cost, stable
links, and clear lifecycle. Folder names are secondary to those properties.

## Conceptual Buckets

| Conceptual bucket     | Current path                        | Audience                  | Lifecycle                     | Owns                                                                 |
| --------------------- | ----------------------------------- | ------------------------- | ----------------------------- | -------------------------------------------------------------------- |
| Published user manual | `docs/readthedocs/`                 | DART users                | durable, published            | Public user docs, tutorials, topic guides, generated API site inputs |
| Developer handbook    | `docs/onboarding/`                  | contributors, agents      | durable, repo-local           | Build/test/CI/release/API policy and landed developer explanations   |
| AI operating model    | `docs/ai/`                          | agents, maintainers       | durable, repo-local           | Agent principles, terminology, workflow routing, gates, sessions     |
| Theory background     | `docs/background/`                  | researchers, implementers | durable, reference            | Physics, math, robotics, control, collision, graphics, theory        |
| Design rationale      | `docs/design/`                      | maintainers, implementers | durable, revisable            | Architecture, API shape, tradeoffs, constraints, accepted decisions  |
| Living roadmap        | `docs/plans/`                       | maintainers, agents       | living, operational           | Priority, dependencies, open gaps, gates, acceptance criteria        |
| Active task handoff   | `docs/dev_tasks/`                   | agents, task owners       | temporary, active             | Multi-session implementation state and resume context                |
| API generation inputs | `docs/doxygen/`, `docs/python_api/` | documentation tooling     | durable, generated-path input | C++ and dartpy API reference inputs                                  |
| Shared assets         | `docs/assets/`                      | docs authors              | durable asset input           | Source-controlled docs assets outside site-local `_static/`          |

## Current Compatibility Contract

`docs/onboarding/` currently acts as the developer handbook. The name remains
for link stability and because many AI workflows and skills already route to
that path. Treat it as a handbook bucket in prose, but do not rename it unless
the repository first has automated link coverage and a migration PR that updates
all command, skill, docs, and site references together.

`docs/plans/` and `docs/dev_tasks/` intentionally overlap in project-management
language, but not in lifecycle:

- `docs/plans/` owns roadmap state and work selection.
- `docs/dev_tasks/` owns branch/session-local implementation handoff state.
- Completed task knowledge moves out of `docs/dev_tasks/` before the task
  folder is deleted.

`docs/design/` and `docs/plans/` intentionally overlap in architecture work, but
not in mutability:

- `docs/design/` owns durable rationale.
- `docs/plans/` owns priority, sequencing, next step, gates, and acceptance
  criteria.

## Placement Matrix

Use this matrix before creating or moving a doc:

| If the content is...                                              | Put it in...                            | Do not put it in...                         |
| ----------------------------------------------------------------- | --------------------------------------- | ------------------------------------------- |
| User-facing install, tutorial, or concept guidance                | `docs/readthedocs/` or root `README.md` | `docs/onboarding/` as the only public path  |
| Contributor workflow, build/test/CI/release operation             | `docs/onboarding/`                      | `docs/ai/` unless the rule is agent-only    |
| AI agent rule, workflow map, terminology, or verification policy  | `docs/ai/`                              | `docs/onboarding/` as the source of truth   |
| Physics, robotics, math, solver, optimization, or graphics theory | `docs/background/`                      | `docs/design/` unless it is a DART decision |
| DART architecture/API decision and rationale                      | `docs/design/`                          | `docs/plans/` if it has no active sequence  |
| Priority, dependency, milestone, active gap, gate, or next step   | `docs/plans/dashboard.md` or plan file  | `docs/design/`                              |
| Multi-session implementation status, branch note, resume prompt   | `docs/dev_tasks/<task>/`                | durable docs after the task completes       |
| Generated API reference source                                    | `docs/doxygen/` or `docs/python_api/`   | handwritten handbook pages                  |
| Shared image or data asset for docs                               | `docs/assets/` or site-local `_static/` | task folders unless temporary               |

## Split And Rename Criteria

Add a new top-level docs folder only when all of these are true:

1. The content has a distinct audience and lifecycle from every existing
   bucket.
2. At least three durable docs would move there immediately.
3. The move reduces required-reading ambiguity for agents.
4. `docs/README.md`, `docs/AGENTS.md`, relevant workflow sources, generated
   adapters, and docs-policy checks can be updated in the same PR.

Rename a top-level docs folder only when all of these are true:

1. The current name repeatedly causes wrong placement or workflow routing.
2. A compatibility plan exists for existing links, skills, commands, and
   published references.
3. The target name is stable enough to keep for multiple release cycles.
4. The rename is verified with Markdown link checks and affected workflow
   adapter sync checks.

Do not split by temporary status, horizon, priority, or north-star dimension.
Those categories move too often and belong in plan/dashboard fields, not paths.

## Recommended Future Shape

The conceptual future shape is:

```text
docs/
├── ai/          # agent operating model and AI workflow policy
├── handbook/    # future name for current docs/onboarding/, if renamed
├── background/  # theory and reference foundations
├── design/      # durable DART technical rationale
├── plans/       # living roadmap and sequencing
├── dev_tasks/   # temporary active task handoff state
├── readthedocs/ # published user manual
├── doxygen/     # C++ API generation input
├── python_api/  # dartpy API reference input
└── assets/      # shared docs assets
```

This is a direction, not an immediate migration. The current path
`docs/onboarding/` remains the handbook bucket until a rename provides enough
value to justify link churn.

## Agent Routing Rules

When a prompt asks where documentation belongs:

1. Read this file, `docs/README.md`, and the local folder `README.md` or
   `AGENTS.md`.
2. Classify the content by lifecycle first, then audience, then topic.
3. Prefer updating an existing owner doc over creating a new file.
4. If a file is temporary task state, put it under `docs/dev_tasks/` and plan
   its deletion path up front.
5. If a file contains mutable priority, next-step, or gate state, route it to
   `docs/plans/` or the dashboard, not `docs/design/`.
6. If a durable learning comes from task completion, promote it to
   `docs/onboarding/`, `docs/design/`, `docs/plans/`, or `docs/readthedocs/`
   before deleting the task folder.

## Verification

For edits to documentation structure or placement policy, run:

- `pixi run lint-md`
- `pixi run check-lint-md`
- `pixi run check-docs-policy`
- `pixi run check-lint-spell`

If the change touches AI workflow sources, generated adapters, or `docs/ai/`,
also run the AI docs/adapters gates from `docs/ai/verification.md`.
