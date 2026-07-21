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
verification commands. Behavior-bearing physics or simulation deliverables
should include a high-quality, self-contained GUI example or durable demo
artifact with runnable command and visual evidence unless explicitly out of
scope. Record review evidence as well: at least two clean independent or
role-separated passes on the current post-fix state.

Add these sidecars when they improve resumability or evidence:

- `decisions.md`: date, decision, context, options considered, evidence,
  tradeoffs, result, and revisit trigger.
- `verification.md`: date, chunk or milestone, what changed, checks run, review
  passes, GUI/demo evidence when relevant, results, known gaps, and follow-up.
- `progress-log.md`: chronological record of meaningful completed work.

## Generated Evidence

Keep generated captures, videos, raw benchmark traces, run logs, and sealed
evidence bundles under `docs/dev_tasks/<task>/assets/`. That directory is local
working state and is ignored by Git. Commit compact result summaries,
reproduction commands, and claim boundaries in the task Markdown files.

If media is useful beyond the active task, promote a deliberately selected,
reasonably sized artifact to `docs/assets/` under the documentation placement
rules. Attach review-only videos through the pull-request editor instead of
adding them to the repository.

Before completing a task, move durable facts to the owner selected by
`docs/information-architecture.md`: published docs, onboarding docs, AI policy,
living plans, design rationale, theory/reference background, reusable docs
assets, API docs, changelog, compatibility notes, or source comments. Do not
leave important decisions only in a temporary task folder.
