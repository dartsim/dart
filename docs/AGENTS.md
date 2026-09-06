# docs/

Agent rules for editing DART documentation.

## Required Reading

1. Root `AGENTS.md`
2. `docs/README.md` (tree map, placement matrix, maintenance rules)
3. The `README.md` or `AGENTS.md` of the bucket you are editing
4. For `docs/ai/`, the read order in `docs/ai/README.md`

## Rules

- Place content by the matrix in `docs/README.md` § "Where Docs Belong":
  lifecycle first, then audience, then topic.
- Prefer rewriting or deleting an existing doc over adding one. A new file
  must be linked from its bucket index in the same change.
- Keep mutable state (status, next step, gates) in `docs/plans/`; keep
  temporary handoff state in `docs/dev_tasks/<task>/`; keep user-facing pages
  in `docs/readthedocs/`.
- Do not append history to active docs; rewrite the current state and let git
  keep the past.
- Do not hand-edit generated adapters; edit `.claude/` sources and run
  `pixi run sync-ai-commands` (see `docs/ai/components.md`).

## Verification

Run the docs gates listed in `docs/README.md` § "Verification"; use
`docs/ai/verification.md` when AI docs or adapters change.
