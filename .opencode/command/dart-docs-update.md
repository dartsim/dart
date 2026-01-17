---
description: Update documentation (no code changes)
agent: build
---

Update documentation: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/README.md

## Workflow

1. `git checkout -b docs/<topic> origin/main`
2. Edit only: `docs/**`, `README.md`, `.claude/`, `.opencode/`
3. `git push -u origin HEAD && gh pr create`
