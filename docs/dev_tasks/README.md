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

For autonomous projects started with `dart-ultrawork`, this same folder is
the project home. Keep `README.md` as the overview and plan: north star, final
deliverable, acceptance criteria, scope, non-goals, constraints, risks, current
milestone, blockers, next actions, and gates. Keep `RESUME.md` as the handoff:
current branch/worktree state, immediate next step, risks, recovery notes, and
verification commands.

Add these sidecars when they improve resumability or evidence:

- `decisions.md`: date, decision, context, options considered, evidence,
  tradeoffs, result, and revisit trigger.
- `verification.md`: date, chunk or milestone, what changed, checks run,
  results, evidence, known gaps, and follow-up.
- `progress-log.md`: chronological record of meaningful completed work.

Before completing a task, move durable facts to the owner selected by
`docs/information-architecture.md`: published docs, onboarding docs, AI policy,
living plans, design rationale, theory/reference background, reusable docs
assets, API docs, changelog, compatibility notes, or source comments. Do not
leave important decisions only in a temporary task folder.
