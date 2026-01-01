# Codex Prompt: DART Examples Overhaul (02)

Use the text below as the prompt for a new Codex chat.

---

You are Codex, working in `/home/js/dev/dartsim/dart/refactor`.

Goal: Continue the multi-phase reorganization of DART C++ examples so they are
user-first, logically ordered, and cover core capabilities. Keep task docs
under `docs/dev_tasks/examples/` up to date so work can resume later.

Read first:
- `AGENTS.md`
- `docs/onboarding/ci-cd.md`
- `docs/onboarding/build-system.md`
- `docs/onboarding/building.md`
- `docs/onboarding/testing.md`
- `docs/onboarding/io-parsing.md`
- `docs/onboarding/gui-rendering.md`
- `docs/onboarding/code-style.md`
- `CONTRIBUTING.md`

Inspect:
- DART examples: `examples/`
- DART tutorials: `tutorials/`
- Reference repos (local):
  - `/home/js/dev/physics_engine/newton/newton/examples`
  - `/home/js/dev/physics_engine/Genesis/examples`

Current state (read in repo):
- Numbered categories now cover the examples tree; uncategorized folders are gone.
- Most examples have metadata headers; some advanced areas still need gap analysis.
- `examples/README.md` lists the category order at a high level.
- See `docs/dev_tasks/examples/01_progress.md` for detailed status and gaps.

Deliverables:
- Update `docs/dev_tasks/examples/01_progress.md` with new checkpoints.
- Update `docs/dev_tasks/examples/00_plan.md` only if taxonomy or criteria change.
- Focus on gap analysis, tutorial cross-links, and any missing metadata.
- Create a commit per checkpoint so progress is resumable.

Constraints:
- Use `pixi run` entry points; do not invent new ones.
- Keep docs concise and avoid hardcoded file lists or performance numbers.
- Default to ASCII.

Focus areas to compare:
- Newton: domain-based categories, assets folder, unified runner with CLI flags, example test hooks.
- Genesis: category folders plus tutorials, performance and sensors coverage, per-folder READMEs where needed.

Definition of done for this phase:
- A clear, user-first example taxonomy and ordering.
- A gap analysis of DART capabilities vs example coverage.
- A phased plan with incremental milestones and updated progress notes.

---
