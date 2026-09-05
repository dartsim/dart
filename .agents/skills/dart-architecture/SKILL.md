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

World owns topology, time and composition. A solver may advance one or several
physical domains, using a shared solve or an explicit coupling strategy.
Semantic dependencies, executable plans and runtime adapters separate physics
from scheduling. Users configure DART-owned method/policy values, never solver
registries, component storage or runtime objects.

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
where implementation is partial: complete shared-model states, full CUDA World
stepping, portable continuation, and coverage beyond the existing metrics/corpus.
It owns the continuous audit rule. New families enter through
`docs/plans/solver-family-intake.md`; `docs/plans/040-dart7-release-hardening.md`
coordinates milestone readiness, and active subsystem plans own packets.
PLAN-091 is completed background. Recheck source evidence before copying a
pattern or repeating an absence claim from an older audit.

## Key owner documents

The architecture page's **Source-of-truth map** is the single owner of the full
topic → owner-doc mapping (solver, API, extension, compute, differentiable,
clean-break, north-star). The docs an agent most often needs inline:

| Topic                                                       | Document                                                                    |
| ----------------------------------------------------------- | --------------------------------------------------------------------------- |
| Solver abstraction, domain assignment, coupling, schedule   | `docs/design/simulation_solver_architecture.md`                             |
| Verified findings, standing rule, competitor lessons        | `docs/design/dart7_architecture_assessment.md`                              |
| Public C++ / dartpy API shape and promotion rules           | `docs/design/simulation_cpp_api.md`, `docs/design/simulation_python_api.md` |
| CPU / SIMD / GPU decision framework                         | `docs/design/scalable_compute_decisions.md`                                 |
| DART 7 vs DART 6 topology · live progress / readiness gates | `docs/design/dart7_clean_break_strategy.md`, `docs/plans/dashboard.md`      |

## Public-facade rules (do not violate)

- Do not expose `Solver`, `Coupler`, `PhysicsDomain`, ECS storage, component
  types, concrete runtime/backend types, or solver registries as public API.
  Existing DART-owned abstract executor/stage extension interfaces are distinct
  from concrete runtime implementations.
- Select behavior by documented method-family names and policy value objects.
- A small DART-owned CPU/CUDA preference is accepted public design; library,
  pool, stream, kernel and ISA types remain private. Existing identifier
  checkers stay enforced until WP-040.2 migrates their focused fixtures.
- Keep the easy path (`World` + `addRigidBody`/`addMultibody` + `step`) free of
  solver vocabulary.
- Fallbacks must never silently substitute algorithms: validate capabilities
  at finalize or record the substitution in diagnostics.

## Verification

Use `docs/ai/verification.md` to select the docs-only or code gate set for the
change. Implementation work that realizes parts of this architecture also
follows the gates in `docs/design/simulation_solver_architecture.md`, the
solver-family intake checklist, and any active owner plan named in
`docs/plans/dashboard.md`.
