---
name: dart-architecture
description: "DART Architecture: the DART 7 multi-physics, multi-solver, multi-backend simulation pipeline and where each abstraction is owned"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-architecture/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART 7 Architecture

Load this skill when working on the experimental simulation `World`, on
solvers/physics domains/compute backends, or whenever a task needs the
big-picture map of how DART 7 is generalized for multi-physics, multi-solver,
and multi-backend simulation.

## The design in one sentence

The `World` owns topology, time, and a configured set of **solvers**; each
solver advances the dynamics of the entities in its **physics domain**, and
**couplers** mediate interactions between domains — all expressed as
**compute-graph** work that any **backend executor** runs. Users configure
method families and policies, never solver registries, component storage, or
execution backends.

## Why three axes of choice

- **Research, apples-to-apples.** A new paper's algorithm should be reproducible
  and benchmarkable _inside_ DART against baselines on shared foundations, not in
  a fork. New methods enter through DART-owned solver _families_.
- **End-user choice.** Users pick the solver method and (internally) backend that
  fit their accuracy/speed/platform needs.
- **Auto-configuration.** Defaults are selected from scene content so the easy
  path stays trivial; the backend seam is designed for later platform/scene-scale
  awareness without changing the public API.

## Visual one-paper (read this first)

`docs/readthedocs/architecture.md` — the single-page map: the pipeline as
abstracted boxes, the available options at each seam (physics domains, solver
families, coupling, collision/contacts, compute graph, executors), status
markers, and the source-of-truth map. Published on the docs site as **Architecture**.

## Key owner documents

The architecture page's **Source-of-truth map** is the single owner of the full
topic → owner-doc mapping (solver, API, extension, compute, differentiable,
clean-break, north-star). The docs an agent most often needs inline:

| Topic                                                     | Document                                                                                              |
| --------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Solver abstraction, domain assignment, coupling, schedule | `docs/design/simulation_solver_architecture.md`                                                       |
| Public C++ / dartpy API shape and promotion rules         | `docs/design/simulation_experimental_cpp_api.md`, `docs/design/simulation_experimental_python_api.md` |
| CPU / SIMD / GPU decision framework                       | `docs/design/scalable_compute_decisions.md`                                                           |
| DART 7 vs DART 6 topology · live progress / parity gates  | `docs/design/dart7_clean_break_strategy.md`, `docs/plans/dashboard.md`                                |

## Public-facade rules (do not violate)

- Do not expose `Solver`, `Coupler`, `PhysicsDomain`, ECS storage, component
  types, executor/backend types, or solver registries as public API.
- Select behavior by documented method-family names and policy value objects.
- Backend names (CUDA, Taskflow, SIMD ISA) may appear in build flags,
  diagnostics, and benchmarks — never in public types, namespaces, or required
  configuration.
- Keep the easy path (`World` + `addRigidBody`/`addMultibody` + `step`) free of
  solver vocabulary.

## Verification

Docs-only edits use the docs-only gate set in `docs/ai/verification.md`
(`pixi run lint`, `pixi run check-docs-policy`, `pixi run docs-build` when the
Read the Docs site changes). Implementation work that realizes parts of this
architecture follows the gates in `docs/design/simulation_solver_architecture.md`.
