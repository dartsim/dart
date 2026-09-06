---
type: ai-north-star
owner: self
---

# AI-Native North Star

This file is the repo-level source of truth for DART's mission and current
project state. Current plan operating state lives in `docs/plans/dashboard.md`,
and plan policy and north-star dimensions live in `docs/plans/README.md`. This
file complements the release roadmap and active dev-task folders; it does not
replace issue tracking, CI, or release notes.

## North Star

DART's north star is to be the dependable, research-focused physics engine that
sets the standard for robotics simulation and animation: accurate
articulated-body dynamics, trustworthy contact and constraint solving, broad
model-format support, and an easy public API backed by a maintainable C++23
core.

Research-focused does not mean research-only. DART should continue to support
production and downstream users on a best-effort basis through release branches,
CI, compatibility policy, and migration paths, while research needs set the
long-term architecture priorities.

Research-focused means three things:

1. DART should be easy to start using. User-facing APIs should be intuitive,
   clean, and small enough for a new researcher to understand quickly. Packages
   should be available through common package managers, and source builds should
   be reproducible through Pixi for users who prefer not to install dependencies
   manually. A first-time user should reach a working simulation with minimal
   setup friction, and the common-path API should be usable from names, types,
   defaults, and short examples without requiring a long user guide first.
2. DART should be easy to extend with new algorithms. Its architecture should
   let researchers add methods from new papers, compare them against existing
   baselines, and reuse shared foundations such as math, collision, memory,
   threading, SIMD, model loading, and tests. The easiest place to reproduce and
   evaluate a new algorithm should be inside DART, not in a one-off fork.
3. DART should scale with the computing platforms researchers actually use.
   Multi-core CPU support is a first-class direction, SIMD is already part of
   the codebase, and DART 7 includes private opt-in CUDA support for selected
   simulation workloads. [PLAN-040](../plans/040-dart7-release-hardening.md)
   owns milestone backend requirements and their acceptance evidence. Broader
   GPU expansion should still be made
   from benchmarkable algorithm needs, packaging impact, security boundaries,
   and maintenance cost. Scalability work should be judged by real research
   workloads, not by backend availability alone.

DART 7 is a new engine, distinct from DART 6. Independent physical oracles,
complete simulation-state continuation and continuously audited model/state,
solver and compute contracts define readiness. The milestone sequence and
implementation prerequisites live in
[PLAN-040](../plans/040-dart7-release-hardening.md). The intended default solver is a
versioned policy over tested method variants/combinations, not a promise of one
universally best algorithm. M1 validates a bounded foundation before expansion.

The intended result is that DART becomes the default reference platform when
researchers evaluate simulation libraries across these three dimensions.

How the DART 7 engine realizes these dimensions — one simulation pipeline
generalized for multi-physics, multi-solver, and multi-backend execution, with
the available options at each abstraction seam — is mapped on a single page in
[`../readthedocs/architecture.md`](../readthedocs/architecture.md), backed by the
design rationale in
[`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
and [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md).

For AI-native work, the north star adds one operational requirement: a fresh
agent should be able to inspect the repository, understand the mission and
current status, choose the next valuable task, implement it locally, and verify
the result with documented gates without needing hidden maintainer context.

## Consumption Modes

DART is used in three ways, which together shape its priorities:

1. **Embedded physics backend (C++ library).** DART supplies articulated-body
   dynamics, contact, and constraint solving as a C++ library inside larger
   simulation frameworks — for example Gazebo through `gz-physics`/`gz-sim`.
   This path is headless and API-only.
2. **Python research toolkit (`dartpy`).** Researchers drive DART from Python
   for robotics, machine learning, and reinforcement learning, headless or with
   lightweight visualization — the role also filled by tools such as MuJoCo,
   Isaac Gym/Sim, Newton, and Genesis.
3. **Standalone GUI simulator (`dartsim`).** The `dartsim` application is a
   full-featured, interactive desktop simulator and scene editor in the spirit
   of game engines and DCC tools (Unity, Unreal Engine, Blender): design, run,
   record, and replay simulations.

These map onto two distribution surfaces:

- `dart/` is the **library** distribution (C++ and Python) serving modes 1 and 2. Its public API and `dartpy` bindings are the compatibility surface for
  downstream consumers.
- `dartsim/` is the **standalone application** distribution serving mode 3. It
  ships as a runtime executable (via package managers), not as a library, so it
  is not consumed by downstream code and keeps minimal runtime dependencies.

<!-- docs-policy: evidence-last-verified=2026-09-05 -->

## Current State

| Area                     | Status                                                                                                                                                                                                                                                                                                                           | Evidence                                                                                                                                                                              |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Release direction        | `main` targets DART 7 as the clean-break line with a Python-first API; the active DART 6 LTS `release-6.*` branch remains the compatibility line for the established DART 6 API and Gazebo/gz-physics users; DART 8 is reserved for post-DART-7 cleanup rather than the active DART 6 removal point.                             | `README.md`, `docs/onboarding/release-roadmap.md`, `docs/design/dart7_clean_break_strategy.md`, `CHANGELOG.md`                                                                        |
| Core engine              | The simulation core is layered around common utilities, math/LCP, dynamics, collision, constraints, simulation, IO, GUI, SIMD, and Python bindings.                                                                                                                                                                              | `docs/onboarding/architecture.md`, module `AGENTS.md` files under `dart/`                                                                                                             |
| Public API modernization | DART 7 uses C++23, snake_case public headers with generated PascalCase compatibility wrappers, explicit API-boundary policy, and the `pixi run check-api-boundaries` policy gate.                                                                                                                                                | `docs/onboarding/code-style.md`, `docs/onboarding/api-boundaries.md`, `scripts/check_api_boundaries.py`                                                                               |
| Python-first API         | `dartpy` is nanobind-based, flattens most user symbols onto the top-level module, keeps temporary legacy warnings only while DART 7 clean-break gates close, and treats Python bindings as a public API filter.                                                                                                                  | `docs/onboarding/python-bindings.md`, `python/AGENTS.md`, `pyproject.toml`                                                                                                            |
| Model loading            | `dart::io::readSkeleton` is the preferred front door for skeleton loading, while parser-specific APIs remain available for advanced cases.                                                                                                                                                                                       | `docs/onboarding/io-parsing.md`, `dart/io/AGENTS.md`                                                                                                                                  |
| Research accessibility   | README installation paths cover Python packages, C++ packages, and Pixi-based source workflows, and the north-star criteria for API simplicity and package readiness are the ones stated above.                                                                                                                                  | `README.md`, `docs/onboarding/building.md`, `pixi.toml`                                                                                                                               |
| Algorithm extensibility  | Modular foundations exist, and PLAN-091 retired the June 2026 architecture-hardening packet plan after landing its planned guardrails, evidence, and ownership cleanup; the architecture assessment remains the current owner for unresolved implementation gaps.                                                                | `docs/design/dart7_architecture_assessment.md`, `docs/plans/solver-family-intake.md`, `docs/plans/dashboard.md`                                                                       |
| Scalable computation     | The codebase includes SIMD and allocator work, CI covers major host platforms, and build options already separate optional accelerator modules; the scalable-compute decision framework and backend evidence survey now exist, while multi-core and GPU implementation is sequenced under PLAN-030.                              | `dart/simd/`, `docs/design/scalable_compute_decisions.md`, `docs/design/compute_backend_research.md`, `docs/plans/dashboard.md`, `dart/simulation/compute/`, `.github/workflows/`     |
| Build, test, and CI      | Pixi tasks provide the contributor path; CI spans Linux, macOS, Windows, FreeBSD, Alt Linux, SIMD, lint, CodeQL, gz-physics, and wheel publishing workflows.                                                                                                                                                                     | `pixi.toml`, `.github/workflows/`, `docs/onboarding/building.md`, `docs/onboarding/testing.md`, `docs/onboarding/ci-cd.md`                                                            |
| AI-native substrate      | Root instructions, AI-infra principles and terminology, workflow maps, tracked session and cross-session context, plan/dev-task states, verification policy, `dart-next`, generated Codex adapters, sync/runtime/scenario checks, commit guards, full validation, and bounded dashboard/archive lifecycle policy are in place.   | `AGENTS.md`, `docs/`, `.claude/commands/`, `.claude/skills/`, `.agents/skills/`, `.codex/`, `scripts/sync_ai_commands.py`, `scripts/install_git_hooks.py`, `docs/dev_tasks/README.md` |
| Active large tasks       | World split is no longer tracked as an active dev-task folder. DART 7 World promotion now lives in PLAN-040/041/042 plus the durable API/design docs; active algorithm work remains in focused `docs/dev_tasks/` folders (GUI work is owned by `docs/design/dartsim_gui_simulator.md`) until each task is completed and retired. | `docs/plans/dashboard.md`, `docs/plans/041-official-simulation-api-promotion.md`, `docs/plans/042-dart7-public-api-and-source-layout.md`, `docs/dev_tasks/README.md`                  |

## What Is Missing

| Gap                                                                        | Why it matters                                                                                                                    | Resolution plan                                                                                                                                                                                                                                                             |
| -------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Current project status is only partly machine-checkable.                   | Link and task-shape checks cannot establish whether a recorded decision or completion claim is still correct.                     | Use `pixi run check-docs-policy` for owner links and task shape; verify current status against the evidence owned by each plan or task before acting.                                                                                                                       |
| Active dev-task hygiene still depends on manual cleanup discipline.        | Long-running work loses context when a task folder omits current status, resume instructions, or a clear promotion/deletion path. | Keep DART 7 World promotion in PLAN-040/041/042 and durable owner docs selected by `docs/README.md`; keep active folders compliant with `docs/dev_tasks/README.md`, promote durable artifacts before deleting completed folders, and add checks if task-shape drift recurs. |
| Release roadmap is product-focused, not execution-focused.                 | It states DART 7 clean-break direction but does not by itself rank the technical work needed to get there.                        | Track current execution order in `docs/plans/dashboard.md` and keep detailed rationale or contributor guidance in the matching `docs/design/`, `docs/onboarding/`, or `docs/readthedocs/` owner.                                                                            |
| Installed research workflows still need end-to-end qualification.          | Self-explanatory APIs, package availability, and Pixi source-build ergonomics determine whether researchers can start quickly.    | PLAN-040/041 own the research-workflow acceptance criteria. Qualify public API clarity, installed C++/Python workflows, complete checkpoints, and package/source-build paths with the required evidence.                                                                    |
| Algorithm extension points need continued intake discipline.               | Researchers need to implement new algorithms and compare them against built-in baselines without rewriting foundations.           | PLAN-091 retired the first hardening packet plan; keep the verified findings in `docs/design/dart7_architecture_assessment.md` current and route every new family through the strengthened `docs/plans/solver-family-intake.md`.                                            |
| Compute scalability still needs workload-specific implementation evidence. | Multi-core CPU and GPU progress must be judged against research workloads and maintenance cost.                                   | Follow PLAN-030 in `docs/plans/dashboard.md` for current implementation order and `docs/design/scalable_compute_decisions.md` for the decision framework; do not duplicate their evolving backlog here.                                                                     |
| Milestone-specific acceptance evidence remains outstanding.                | Passing a broad gate is only useful when it covers the actual objective.                                                          | PLAN-040 and its owning packets define objective-specific acceptance. Execute those evidence contracts and the change-type gates in `docs/ai/verification.md`; documented criteria alone do not complete a milestone.                                                       |

## Planning Surfaces

`docs/plans/dashboard.md` owns the operating state of every open plan;
`docs/plans/README.md` owns the plan lifecycle and north-star dimensions;
`docs/README.md` owns which bucket holds each other kind of knowledge.

## Autonomous Agent Loop

1. Read `AGENTS.md`, then this file, then the task-specific docs named by
   `AGENTS.md`.
2. Choose work only from concrete evidence: active dev tasks, release roadmap,
   issues, PR/CI state, or explicit user instruction.
3. Keep the task bounded to one branch and one verification story.
4. Make local edits and run the strongest relevant gates.
5. Update durable docs when the project state or path changes.
6. Pushes, PR comments, review-thread changes, CI re-triggers, and merges need
   explicit maintainer/user approval (`docs/ai/principles.md`, axiom 9).

## Completion Criteria For AI-Native Readiness

DART is ready for sustained autonomous agent work when:

- a fresh agent can state the mission, current status, and next execution path
  from tracked docs alone;
- active long-running work is represented by compliant dev-task folders or by
  durable owner docs selected by `docs/README.md`;
- each major roadmap item names its verification evidence;
- research-facing APIs are clean, package/source-build paths are verified, and
  new algorithms can be benchmarked against existing baselines;
- compute scalability work has an evidence-backed CPU/GPU roadmap, including
  M1 runtime qualification and small DART-owned device preferences while
  runtime/library types stay private;
- `dart-next` or an equivalent workflow stays active across supported AI
  surfaces and can select one bounded next task without hidden maintainer
  context;
- generated AI adapters and documentation checks keep the workflow surfaces in
  sync.
