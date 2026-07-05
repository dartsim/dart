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

## Current Diagnosis

The original `docs/onboarding/` goal was a wiki-like developer knowledge base:
humans and AI agents should understand DART from tracked docs instead of
rediscovering architecture, workflow, and project state from raw code. That
goal is still correct, but the repository has grown additional doc buckets with
different lifecycles:

- `docs/ai/` is no longer just another handbook chapter. It owns the agent
  operating model: principles, terminology, workflow routing, sessions, and
  verification policy.
- `docs/background/` is reference material for theory and original-source
  foundations. It should grow by topic, but it should not become the place for
  DART-specific architecture decisions.
- `docs/design/` is durable DART engineering rationale: architecture, API
  shape, tradeoffs, constraints, and accepted decisions.
- `docs/plans/` is living roadmap state: priority, dependency, horizon, open
  gaps, next step, and gates.
- `docs/dev_tasks/` is temporary implementation handoff state for active
  multi-session work. Completed knowledge must move out before the task folder
  is deleted.
- `docs/readthedocs/`, `docs/doxygen/`, and `docs/python_api/` are published
  documentation and API-generation inputs, not the internal developer handbook.

The scalable structure is therefore lifecycle-first. Topic-first trees such as
`docs/physics/`, `docs/rendering/`, or `docs/release/` are tempting, but they
would mix user guides, theory, design rationale, roadmap state, and task
handoff in one place. That would increase the context cost for agents and make
cleanup rules harder to enforce.

## Restructure Decision

The best current structure is to keep the existing top-level buckets, clarify
their lifecycle boundaries, and treat `docs/onboarding/` as the current
developer handbook path in prose. Do not perform a broad physical migration in
the same change as this policy clarification.

Recommended actions:

1. Keep `docs/onboarding/` as the tracked path for durable developer handbook
   material until a dedicated compatibility migration justifies a rename.
2. Use `docs/information-architecture.md` as the placement owner instead of
   scattering placement rules across each directory README.
3. Keep root pointer boards (`docs/README.md`, `docs/AGENTS.md`, root
   `AGENTS.md`) short and route readers to this file for placement decisions.
4. Prefer sidecar subdirectories inside an existing bucket when one initiative
   outgrows a single file. Add a new top-level folder only when the split meets
   the criteria below.
5. Use policy checks to keep the placement owner discoverable, then rely on
   local folder READMEs and AGENTS files for bucket-specific details.

## Alternatives Considered

| Alternative                                                  | Verdict                 | Reason                                                                                                                                          |
| ------------------------------------------------------------ | ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| Rename `docs/onboarding/` to `docs/handbook/` immediately    | Defer                   | The name is better, but the current path is referenced by many docs, workflows, skills, and checks. Rename only through a dedicated migration.  |
| Add `docs/wiki/` or `docs/knowledge/`                        | Reject                  | It would recreate the original ambiguity as a catch-all bucket and compete with handbook, design, background, and plans.                        |
| Merge `docs/ai/` back into `docs/onboarding/`                | Reject                  | AI workflow policy changes faster and has tool-specific safety rules; it needs a compact agent entrypoint.                                      |
| Move tool compatibility from `docs/onboarding/` into `ai/`   | Defer unless it narrows | Tool compatibility is partly contributor workflow and partly AI policy. Keep the owner where current workflows point unless it becomes AI-only. |
| Merge `docs/plans/` and `docs/dev_tasks/`                    | Reject                  | Plans choose and sequence work; dev tasks preserve branch/session handoff. Merging them would blur cleanup rules.                               |
| Merge `docs/background/` into `docs/design/`                 | Reject                  | Theory/reference material and DART engineering decisions have different attribution, edit, and verification rules.                              |
| Split `docs/design/` by subsystem at the top level           | Reject for now          | Design docs are still navigable by index. Use sidecar directories for large design families before adding top-level buckets.                    |
| Move all user-facing material into `docs/readthedocs/`       | Partially accept        | Published user docs belong there, but repo-local contributor and maintainer workflow docs should remain in the handbook.                        |
| Add `docs/research/` for papers, algorithms, and experiments | Reject for now          | Current homes are clearer: theory in `background`, catalogs in published docs, algorithm decisions in `design`, sequencing in `plans`.          |

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
