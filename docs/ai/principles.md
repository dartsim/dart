# AI Principles

These principles apply to AI-assisted work on the DART 6.20 support branch.
Keep this file short: it spends always-loaded agent context, so put procedures
and compatibility detail in the owner docs named by `AGENTS.md`.

## Evidence First

- Inspect the current branch, worktree, and relevant source files before
  editing.
- Treat live GitHub state and current command output as authoritative.
- Do not rely on stale branch notes when a current checkout or PR can be
  inspected.
- Before substantial or consequential work, read enough release-branch context
  to understand the real invariant: `AGENTS.md`,
  `docs/information-architecture.md`, owner docs, affected modules and call
  paths, current release/PR state, and the verification bar. Use that context to
  choose the smallest compatibility-preserving change, not an ad hoc local
  patch or special case. Keep context proportionate for tiny fixes.
- Before a non-trivial fix, surface your unknowns rather than coding a guess:
  your plan is a map, not the territory of the real code. Convert consequential
  unknowns into knowns first — reproduce the bug, read the affected code, or get
  an independent blind-spot review — instead of discovering them mid-change.
- Back consequential decisions with proportionate evidence: code inspection,
  logs, focused tests, benchmarks, source research, or prototypes according to
  the claim. Do not decide major workflow or compatibility choices from
  intuition when direct evidence is practical.
- For unexpected in-scope failures, reproduce the smallest failing case, fix the
  underlying cause, and add regression coverage. Preserve the real invariant at
  the right owner doc or module boundary; do not suppress logs, loosen checks,
  skip cases, or route around symptoms. For pre-existing or external failures,
  classify them with evidence and report or park them without weakening gates.

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
  the owner selected by `docs/information-architecture.md` before retiring a
  task folder.
