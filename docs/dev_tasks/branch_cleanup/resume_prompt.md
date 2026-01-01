# Resume Prompt - Branch Cleanup

You are a Codex agent working in `/home/js/dev/dartsim/dart/cmake_build`.

Goal: triage stale branches in dartsim/dart and decide delete, rebase + PR, or
research + plan. Use origin/main as the target branch (DART 7). origin/release-7.0
is scheduled for removal after dependent branches are resolved.

Required docs:

- Read AGENTS.md.
- Read docs/onboarding/ci-cd.md and docs/onboarding/build-system.md.
- Read CONTRIBUTING.md.
- Read docs/dev_tasks/branch_cleanup/README.md,
  docs/dev_tasks/branch_cleanup/00_plan.md, and
  docs/dev_tasks/branch_cleanup/01_progress.md.

Current status:

- origin/7/fix_archlinux was deleted after confirming its fixes already exist on
  origin/main.

Workflow:

1. Run git fetch --all --prune.
2. For each branch to review:
   - Confirm it exists on origin.
   - Determine original base branch; compare to that base first if it is not
     origin/main.
   - Compute ahead/behind vs origin/main.
   - If ahead is 0: delete the remote branch after confirmation.
   - If ahead > 0: inspect commits, diff, and cherry results; decide delete,
     rebase + PR, or research + plan.
3. If changes are needed, work in a new cleanup branch off origin/main.
4. Update docs/dev_tasks/branch_cleanup/01_progress.md after each batch.
5. Ask before deleting if ownership is unclear.

Deliverables:

- Per-branch decision and rationale.
- Branches deleted (remote).
- PR URLs and CI status.
- Open questions or risks.

If any instructions conflict, ask the user before proceeding.
