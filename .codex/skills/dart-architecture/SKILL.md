---
name: dart-architecture
description: "DART Architecture: the DART 7 multi-physics, multi-solver, multi-backend simulation pipeline and where each abstraction is owned"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-architecture/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART 7 Architecture

Load this skill when working on the DART 7 simulation `World`, on
solvers/physics domains/compute backends, or whenever a task needs the
big-picture map of how DART 7 is generalized for multi-physics, multi-solver,
and multi-backend simulation.

## The design in one sentence

The `World` owns topology, time, and a configured set of **solvers**; each
solver advances the dynamics of the entities in its **physics domain**, and
**couplers** mediate interactions between domains — with parallelizable work
expressed as **compute-graph** nodes that any **backend executor** runs.
Users configure method families and policies, never solver registries,
component storage, or execution backends.

## Why three axes of choice

- **Research, apples-to-apples.** A new paper's algorithm should be reproducible
  and benchmarkable _inside_ DART against baselines on shared foundations, not in
  a fork. New methods enter through DART-owned solver _families_.
- **End-user choice.** Users pick the solver method and (internally) backend that
  fit their accuracy/speed/platform needs.
- **Auto-configuration.** Defaults are selected from scene content so the easy
  path stays trivial; the backend seam is designed for later platform/scene-scale
  awareness without changing the public API.

## Design vs current state (read both)

`docs/readthedocs/architecture.md` is the single-page map of the design and
the options at each seam, with honest status markers.
`docs/design/dart7_architecture_assessment.md` is the verified record of
where the implementation still diverges from that design (no internal solver
contract yet, conceptual-only Model/State split, executor seam unused by
dynamics stages, missing apples-to-apples substrate) and owns the standing
rule: new solver families enter only through
`docs/plans/solver-family-intake.md`, including contract conformance and
machine-recorded solver identity in all benchmark evidence. The fixes are
executed as PLAN-091 work packets
(`docs/plans/091-architecture-hardening.md`) under the orchestrator/executor
model in `docs/ai/orchestration.md`. Do not write new code that copies a
pattern the assessment lists as a verified finding.

## Key owner documents

The architecture page's **Source-of-truth map** is the single owner of the full
topic → owner-doc mapping (solver, API, extension, compute, differentiable,
clean-break, north-star). The docs an agent most often needs inline:

| Topic                                                     | Document                                                                    |
| --------------------------------------------------------- | --------------------------------------------------------------------------- |
| Solver abstraction, domain assignment, coupling, schedule | `docs/design/simulation_solver_architecture.md`                             |
| Verified findings, standing rule, competitor lessons      | `docs/design/dart7_architecture_assessment.md`                              |
| Public C++ / dartpy API shape and promotion rules         | `docs/design/simulation_cpp_api.md`, `docs/design/simulation_python_api.md` |
| CPU / SIMD / GPU decision framework                       | `docs/design/scalable_compute_decisions.md`                                 |
| DART 7 vs DART 6 topology · live progress / parity gates  | `docs/design/dart7_clean_break_strategy.md`, `docs/plans/dashboard.md`      |

## Public-facade rules (do not violate)

- Do not expose `Solver`, `Coupler`, `PhysicsDomain`, ECS storage, component
  types, executor/backend types, or solver registries as public API.
- Select behavior by documented method-family names and policy value objects.
- Backend names (CUDA, Taskflow, SIMD ISA) may appear in build flags,
  diagnostics, and benchmarks — never in public types, namespaces, or required
  configuration.
- Keep the easy path (`World` + `addRigidBody`/`addMultibody` + `step`) free of
  solver vocabulary.
- Fallbacks must never silently substitute algorithms: validate capabilities
  at finalize or record the substitution in diagnostics.

## Verification

Use `docs/ai/verification.md` to select the docs-only or code gate set for the
change. Implementation work that realizes parts of this architecture also
follows the gates in `docs/design/simulation_solver_architecture.md` and the
packet gates in `docs/plans/091-architecture-hardening.md`.
