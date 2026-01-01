# Branch Cleanup - Dev Task

## Status

- In progress: triaging stale branches against origin/main (DART 7).

## Goal

Triage stale branches in dartsim/dart and decide delete, rebase + PR, or
research + plan, using origin/main as the target branch.

## Non-goals

- Drive-by refactors or behavior changes
- Deleting branches without confirmation when ownership is unclear
- Rebasing large or risky branches without a plan

## Constraints

- Use existing tooling and follow docs/onboarding and CONTRIBUTING.md.
- Track decisions and progress in the task docs after each batch.
- When changes are needed, work in a new cleanup branch off origin/main.
- origin/release-7.0 is scheduled for removal after dependent branches are
  resolved.

## Documents

- Plan: docs/dev_tasks/branch_cleanup/00_plan.md
- Progress: docs/dev_tasks/branch_cleanup/01_progress.md
- Resume prompt: docs/dev_tasks/branch_cleanup/resume_prompt.md
