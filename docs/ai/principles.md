# AI Principles

These principles apply to AI-assisted work on the DART 6.20 support branch.

## Evidence First

- Inspect the current branch, worktree, and relevant source files before
  editing.
- Treat live GitHub state and current command output as authoritative.
- Do not rely on stale branch notes when a current checkout or PR can be
  inspected.
- Before a non-trivial fix, surface your unknowns rather than coding a guess:
  your plan is a map, not the territory of the real code. Convert consequential
  unknowns into knowns first — reproduce the bug, read the affected code, or get
  an independent blind-spot review — instead of discovering them mid-change.

## Compatibility First

- DART 6.20 is a compatibility support lane. Preserve existing public headers,
  package components, and downstream Gazebo/gz-physics behavior unless a
  maintainer explicitly accepts a breaking change.
- Bug fixes that apply to both DART 6 and DART 7 should follow the dual-PR
  policy in `docs/onboarding/contributing.md`.

## Approval Boundaries

- GitHub mutations, pushes, PR creation or updates, branch deletion, CI reruns,
  and review-thread mutations require explicit maintainer/user approval.
- Never reply to AI-generated review comments. Make local fixes silently and
  request a fresh review only after an approved push.

## Scope Control

- Keep PRs focused. For dependency minimization, prefer one dependency or one
  `dart/external` tree per PR unless a shared mechanical step is required.
- Use `docs/dev_tasks/<task>/` for multi-session state. Promote durable facts to
  onboarding, release, or compatibility docs before retiring a task folder.
