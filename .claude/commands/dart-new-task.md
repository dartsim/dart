---
description: start a feature, bugfix, refactor, docs, build, or test task
agent: build
---

Start a new task in DART: $ARGUMENTS

## Required Reading

Read these files first:
@AGENTS.md
@docs/onboarding/building.md
@docs/onboarding/contributing.md
@docs/onboarding/code-style.md
@docs/dev_tasks/README.md
@docs/ai/sessions.md
@docs/ai/principles.md
@docs/ai/verification.md

## Workflow

1. **Understand the task** - Parse: goal, constraints, type (feature|bugfix|refactor|docs)
2. **Assess scope** - Multi-phase or multi-session? Create
   `docs/dev_tasks/<task>/` (see `docs/dev_tasks/README.md` for criteria).
   For multi-session, design-heavy, public API, solver/paper, release, or
   cross-module work, fill the dev-task specification intake before editing:
   value, scope, assumptions, traceability, non-goals, acceptance evidence,
   gates, and open decisions. If consequential ambiguity would change public
   API, release compatibility, numerical correctness, benchmark claims, or
   roadmap scope, record an owner-local `Decision needed` block instead of
   silently choosing.
3. **Setup** - Choose the target branch before creating a topic branch. For
   DART 6.20 maintenance, dependency-minimization, docs, and compatibility
   work, branch from `origin/release-6.20` without tracking the release ref:
   `git switch --no-track -c <type>/<topic> origin/release-6.20`. Bug fixes
   that also apply to DART 7 still need a separate `main` PR after the release
   branch fix.
4. **Implement** - Keep commits focused, follow code style
5. **Verify** - Run `pixi run lint` before committing, then run the focused
   release-branch gate for the touched surface. Use `pixi run test-all` when
   feasible, and `pixi run -e gazebo test-gz` for package, collision,
   constraint, or downstream Gazebo/gz-physics compatibility work.
6. **PR** - After explicit maintainer/user approval, push with the same local
   and remote topic-branch name:
   `branch=$(git branch --show-current); git push -u origin "HEAD:${branch}"`.
   Then create the draft PR against `<target-branch>` with the branch-matching
   DART 6.x patch milestone and `.github/PULL_REQUEST_TEMPLATE.md`.
7. **Cleanup** - Before PR: if task used `docs/dev_tasks/<task>/`, first
   promote durable dashboards, evidence matrices, API inventories, migration
   maps, or long-lived decisions into release/onboarding/AI docs.
   Then remove the dev-task folder completely (include the deletion in this PR,
   not after merge).

## Type-Specific

- **Bugfix**: Requires PRs to BOTH the active DART 6 LTS branch AND `main`
- **Refactor**: No behavior changes
- **Feature**: Add tests + docs
- **DART 7 solver or architecture work**: Do that on `main`, not on the DART
  6.20 support branch. Use this release branch only for compatible maintenance,
  dependency minimization, CI, docs, and backports.
