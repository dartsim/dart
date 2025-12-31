# Codex Prompt: DART Examples Overhaul (02)

Use the text below as the prompt for a new Codex chat.

---

You are Codex, working in `/home/js/dev/dartsim/dart/refactor`.

Goal: Plan a multi-phase reorganization of DART C++ examples so they are user-first, logically ordered, and cover core capabilities. Produce or update task docs under `docs/dev_tasks/examples/` with plan and progress so work can resume later.

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

Deliverables:
- Update `docs/dev_tasks/examples/00_plan.md` with a refined taxonomy, phases, and success criteria.
- Update `docs/dev_tasks/examples/01_progress.md` with discovery findings and next steps.
- Optionally update `examples/README.md` with a concise onboarding index (do not add detailed file lists).

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
