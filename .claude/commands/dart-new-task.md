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

## Workflow

1. **Understand the task** - Parse: goal, constraints, type (feature|bugfix|refactor|docs)
2. **Assess scope** - Multi-phase or multi-session? Create `docs/dev_tasks/<task>/` (see `docs/dev_tasks/README.md` for criteria)
3. **Setup** - Choose the target branch before creating a topic branch:
   - features/docs/non-bugfix refactors: branch from `origin/main`
   - bug fixes that apply to the current release line: branch from
     `origin/release-6.16` first, then cherry-pick or reapply to `main`
4. **Implement** - Keep commits focused, follow code style
5. **Verify** - Run `pixi run lint` before committing, then
   `pixi run test-all`; on Linux hosts with a visible NVIDIA CUDA runtime, also
   run `pixi run -e cuda test-all`
6. **PR** - After explicit maintainer/user approval, `git push -u origin HEAD`
   then `gh pr create --draft --base <target-branch> --milestone "<milestone>"`
   (`DART 7.0` for `main`, `DART 6.16.x` for `release-6.16`); follow
   `.github/PULL_REQUEST_TEMPLATE.md`
7. **Cleanup** - Before PR: if task used `docs/dev_tasks/<task>/`, first
   promote durable dashboards, evidence matrices, API inventories, migration
   maps, or long-lived decisions into `docs/plans/` or `docs/onboarding/`.
   Then remove the dev-task folder completely (include the deletion in this PR,
   not after merge).

## Type-Specific

- **Bugfix**: Requires PRs to BOTH `release-6.16` AND `main`
- **Refactor**: No behavior changes
- **Feature**: Add tests + docs
- **New solver/paper implementation**: Derive an evidence matrix from the
  paper, project page, reference source, videos, and demos. Do not call the task
  complete until DART implements all algorithms/features on required CPU and GPU
  backends, ports all experiments/demos into tests/benchmarks/py-demos, records
  benchmark JSON proving DART beats reference and paper numbers for every
  claimed case, and performs any clean API/pipeline refactor needed for the
  long-term DART 7/8 architecture.
