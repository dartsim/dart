# DART Documentation

`docs/` serves DART users, contributors, maintainers, and AI agents. This file
is the map of the tree and the owner of placement policy: read it before
adding, moving, or deleting a doc. The published user site is built from
[`readthedocs/`](readthedocs/README.md); everything else is repository-local
Markdown optimized for GitHub and agent reading.

## Start Here

For people:

| I want to...                            | Read                                                                                                                               |
| --------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| Use DART from C++ or Python             | [dart.readthedocs.io](https://dart.readthedocs.io/) (source: [`readthedocs/`](readthedocs/README.md)); [`examples/`](../examples/) |
| See what the project is working on      | [`plans/dashboard.md`](plans/dashboard.md) (operating view), [`ai/north-star.md`](ai/north-star.md) (mission and current state)    |
| Build, test, or contribute              | [`onboarding/README.md`](onboarding/README.md) (developer handbook), [`CONTRIBUTING.md`](../CONTRIBUTING.md)                       |
| Understand the DART 7 architecture      | [`readthedocs/architecture.md`](readthedocs/architecture.md) (one page), [`design/README.md`](design/README.md) (rationale)        |
| Understand the theory behind the engine | [`background/README.md`](background/README.md)                                                                                     |
| Cut a release or write release notes    | [`onboarding/release-management.md`](onboarding/release-management.md), [`onboarding/changelog.md`](onboarding/changelog.md)       |

For agents: the root [`AGENTS.md`](../AGENTS.md) is the entrypoint. It loads
[`ai/principles.md`](ai/principles.md) for every session and routes each task
type to its owner docs; [`ai/README.md`](ai/README.md) owns the AI operating
model and read order, and [`ai/workflows.md`](ai/workflows.md) owns the
workflow catalog. When a task edits docs, read this file, then
the `README.md` or `AGENTS.md` of the bucket being edited.

## Buckets

Buckets are split by lifecycle first, then audience. Every Markdown bucket
has one index (`README.md`); its docs are reachable from that index, from
`plans/dashboard.md` for numbered plans, or by directory listing for
`dev_tasks/<task>/`.

| Path                                                          | Audience                  | Lifecycle                     | Owns                                                                                      |
| ------------------------------------------------------------- | ------------------------- | ----------------------------- | ----------------------------------------------------------------------------------------- |
| [`readthedocs/`](readthedocs/README.md)                       | DART users                | durable, published            | Public user guide, tutorials, topic pages, research catalog, generated API site inputs    |
| [`onboarding/`](onboarding/README.md)                         | contributors, agents      | durable, repo-local           | Developer handbook: build, test, CI, release, API policy, module explanations, AI tooling |
| [`ai/`](ai/README.md)                                         | agents, maintainers       | durable, repo-local           | AI operating model: principles, north star, workflow catalog, verification, terminology   |
| [`design/`](design/README.md)                                 | maintainers, implementers | durable, revisable            | DART architecture and API decisions with rationale, tradeoffs, constraints                |
| [`background/`](background/README.md)                         | researchers, implementers | durable, reference            | Theory derived from the original DART papers (attributed, PDFs authoritative)             |
| [`plans/`](plans/README.md)                                   | maintainers, agents       | living, operational           | Roadmap state: priority, status, next step, gates, work packets, completed-plan archive   |
| [`dev_tasks/`](dev_tasks/README.md)                           | agents, task owners       | temporary per task            | Multi-session implementation handoff; the folder is deleted when the task completes       |
| [`doxygen/`](doxygen/), [`python_api/`](python_api/index.rst) | docs tooling              | durable, generated-path input | C++ and dartpy API reference inputs                                                       |
| [`assets/`](assets/README.md)                                 | docs authors              | durable asset input           | Shared images and data used by repository docs outside the site-local `_static/`          |

`onboarding/` is the developer handbook; the directory name predates that role
and is kept for link stability. Rename it only through a dedicated migration
that updates every command, skill, script, and site reference in one change.

## Where Docs Belong

Classify content by lifecycle, then audience, then topic, and prefer updating
an existing owner doc over creating a file:

| If the content is...                                                   | Put it in                                 | Not in                                       |
| ---------------------------------------------------------------------- | ----------------------------------------- | -------------------------------------------- |
| User-facing install, tutorial, concept, or API guidance                | `readthedocs/` or the root `README.md`    | `onboarding/` as the only public path        |
| Contributor workflow, build/test/CI/release operation, module overview | `onboarding/`                             | `ai/` unless the rule applies only to agents |
| Agent rule, workflow routing, terminology, verification policy         | `ai/`                                     | `onboarding/` as the source of truth         |
| Physics, math, solver, optimization, or graphics theory                | `background/`                             | `design/` unless it records a DART decision  |
| DART architecture or API decision and its rationale                    | `design/`                                 | `plans/` unless it has an active sequence    |
| Priority, dependency, milestone, open gap, gate, or next step          | `plans/dashboard.md` or the numbered plan | `design/`, `onboarding/`, dev-task folders   |
| Multi-session status, branch note, resume prompt                       | `dev_tasks/<task>/`                       | any durable bucket                           |
| Generated API reference source                                         | `doxygen/` or `python_api/`               | handwritten handbook pages                   |
| Shared image or data asset                                             | `assets/` or site-local `_static/`        | task folders unless temporary                |

Two intentional overlaps:

- `design/` and `plans/` both touch architecture work. `design/` owns durable
  rationale; `plans/` owns sequencing, next step, gates, and acceptance
  criteria.
- `plans/` and `dev_tasks/` both track work. `plans/` chooses and sequences it;
  `dev_tasks/<task>/` holds temporary branch and session handoff state.

## Keeping Docs Current

Docs are maintained as part of the change that makes them stale, not in
periodic cleanups. Git history owns the past; active docs describe the current
state and the remaining work.

| When...                                    | Do this in the same change                                                                                                                             |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| A task using `dev_tasks/<task>/` completes | Promote durable decisions, matrices, and inventories to their owners, redirect links, delete the folder ([`dev_tasks/README.md`](dev_tasks/README.md)) |
| A plan completes                           | Move its dashboard entry to [`plans/archive.md`](plans/archive.md); delete or fold the numbered plan file once nothing points to it                    |
| A `Decision needed` block is resolved      | Replace the block with the decision and rationale in the owner doc; update the dashboard if plan state changed                                         |
| A doc is superseded or duplicated          | Delete it and redirect links; do not leave stub or "deprecated" pages                                                                                  |
| A rule changes                             | Edit the owner doc only; entrypoints and workflows carry pointers, not copies                                                                          |
| A workflow, skill, or AI doc changes       | Follow [`ai/components.md`](ai/components.md) and regenerate adapters                                                                                  |
| A published page is ported to DART 7       | Remove its legacy notice and update [`onboarding/dart7-docs-migration.md`](onboarding/dart7-docs-migration.md)                                         |

Rules that keep the set small:

- **No running logs in active docs.** Dev-task `README.md` and `RESUME.md`
  are snapshots: rewrite them, do not append. Session-by-session history goes
  to git; a numbered plan may keep a bounded `## Progress log` that is pruned
  when its entries no longer inform the next step.
- **One owner per fact.** Mutable state (status, next step, gate, version,
  branch name) lives in exactly one place; other docs link to it. Generated
  files name their source.
- **Every doc is reachable.** A doc that no index or owner references is dead
  weight: link it from its bucket index or delete it.
- **Size budgets.** Dashboard entries stay at or under 40 lines with a next
  step at or under 15 (enforced). A dev-task `RESUME.md` over 200 lines or `README.md`
  over 400 lines is reported as an advisory and should be trimmed.

Run the audit when finishing a task, archiving a plan, or before a release:

```bash
pixi run check-docs-policy   # indexes, links, orphans, plan and dev-task shape, budgets
```

`/dart-docs-update audit` (Claude Code) or `$dart-docs-update audit`
(Codex) runs that check, reviews its advisories, and walks the table above to
propose consolidations and deletions.

## Adding Or Renaming Buckets

Add a top-level folder only when the content has a distinct audience and
lifecycle from every existing bucket, at least three durable docs move there
immediately, and this file, the affected workflows, generated adapters, and
`scripts/check_docs_policy.py` change in the same PR. Rename a folder only with
a compatibility plan for every link, skill, command, and published reference,
verified by the link and adapter checks. Never split by status, horizon,
priority, or north-star dimension; those values move too often.

Structures already considered and rejected, so they are not re-proposed
without new evidence: a catch-all `docs/wiki/` or `docs/knowledge/` bucket
(recreates the placement ambiguity); merging `ai/` back into `onboarding/`
(agent policy changes faster and needs a compact entrypoint); merging `plans/`
with `dev_tasks/` (blurs the cleanup rules); merging `background/` into
`design/` (different attribution and edit rules); topic-first trees such as
`docs/physics/` or `docs/research/` (mix lifecycles in one place); and
splitting `design/` by subsystem at the top level (use sidecar directories
first).

## Verification

For docs-structure or placement changes run:

```bash
pixi run lint-md
pixi run check-lint-md
pixi run check-docs-policy
pixi run check-lint-spell
```

If the change touches AI workflow sources, generated adapters, or `ai/`, also
run the AI docs/adapters gates from [`ai/verification.md`](ai/verification.md).
