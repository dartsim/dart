# docs/background/

Agent rules for DART 6.20 theory and research background.

## Purpose

`docs/background/` owns durable theory, math, paper, and reference notes that
help agents understand DART 6 implementation work. It is not the place for
DART-specific design decisions, roadmap priority, or active handoff state.

## Rules

- Start with `docs/README.md`, `docs/information-architecture.md`, and
  `docs/background/README.md`.
- Preserve attribution for material derived from papers, textbooks, PDFs, or
  external projects.
- Keep release-branch claims compatibility-aware. DART 7 references may be
  useful evidence, but DART 6 changes still need direct release-branch proof.
- Put DART architecture and API tradeoffs in `docs/design/`.
- Put priority, next steps, and acceptance gates in `docs/plans/`.
- Put active implementation handoff state in `docs/dev_tasks/<task>/`.

## Verification

Use `docs/ai/verification.md` to select gates. For background-only Markdown
edits, run `pixi run lint`.
