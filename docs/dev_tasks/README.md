# Development Task Tracking

Use `docs/dev_tasks/<task>/` for multi-session DART 6.20 work that needs
handoff state, sequencing, or evidence beyond a single commit.

Create a task folder when work is multi-phase, spans more than one session, or
has compatibility gates that future agents must preserve.

Recommended files:

- `README.md`: scope, current evidence, decisions, validation, and remaining
  work.
- `RESUME.md`: exact next steps for a later session.
- `HANDOFF.md`: optional transfer note when handing work to another agent or
  worktree.

Before completing a task, move durable facts to the owner selected by
`docs/information-architecture.md`: published docs, onboarding docs, AI policy,
API docs, changelog, compatibility notes, or source comments. Do not leave
important decisions only in a temporary task folder.
