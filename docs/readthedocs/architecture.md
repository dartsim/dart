# DART 7 Architecture: Multi-Physics, Multi-Solver, Multi-Backend

```{note}
DART 7 is a new simulation engine with the promoted `dart::simulation::World`
and `dartpy.World` facade. DART 6 remains a separate maintained compatibility
line on `release-6.*`. This page distinguishes current implementation from the
architecture being qualified; an available kernel or selector does not imply
complete simulation coverage.
```

## Design And Current State

World owns topology, time and simulation composition. A solver may advance one
physical domain or several coupled domains. Interactions can be part of a shared
solve or use an explicit coupling strategy. Users work with bodies, controls,
state and small policy values; internal solver objects, ECS and runtime types
stay hidden.

Physical domain, intrinsic dimension, discretization, numerical method,
coupling and compute backend are separate axes. Articulation is a representation
of rigid physics; particles can discretize several domains. A shell surface and
a surface bounding a volumetric solid need different physical models even when
their input geometry looks similar.

```text
WORLD FACADE — construct · control · step · inspect · checkpoint
  Current: DART-owned method options and resolved configuration
  Planned: versioned selection policy over tested compatible methods
                         │
PHYSICAL SCENE — domains + dimensions + representations + interactions
  Current: rigid/free and articulated paths; bounded deformable paths
  Planned: qualified bidirectional coupling; thin structures and fluid models
                         │
SOLVER PORTFOLIO — each method owns a declared supported envelope
  Current: SI / boxed-LCP contact; rigid IPC; VBD / AVBD;
           articulated semi-implicit / variational; deformable methods
  Planned: versioned variants and validated shared/partitioned combinations
                         │
DATA AND COLLISION — model · state · controls · contacts · continuation
  Current: split components, dense rigid/multibody model, native collision,
           metrics, replay and bounded binary snapshots
  Partial: independent shared-model states and complete portable checkpoints
                         │
COMPUTE — semantic dependency graph → executable plan → runtime adapter
  Current: ordered World stages, explicit graphs within selected stages,
           Taskflow CPU execution, selected resident CUDA kernels
  Planned: qualified ownership/completion and full CUDA rigid/contact step pipeline
```

The arrows show responsibilities, not a claim that the whole World step is
already one executable DAG. The current built-in schedule is an ordered stage
list with preflight/prepare/execute contracts. Stages may emit graphs. A task
executor schedules supplied operations; device execution needs actual device
kernels and correct memory/completion handling.

## Available Options And Their Limits

| Area                     | Current implementation                                                                                        | Remaining qualification                                                                                                      |
| ------------------------ | ------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| Rigid bodies             | `World::addRigidBody`; SI default and opt-in IPC/VBD/AVBD choices                                             | General rotation, isotropic ground friction, complete CUDA rigid/contact stepping                                            |
| Articulated rigid bodies | `World::addMultibody`; semi-implicit and opt-in variational integration, shared rigid/articulated constraints | Complete admitted robotics/control/loading/restart workflow and measured complexity                                          |
| Deformables              | `World::addDeformableBody`; bounded mass-spring/FEM/IPC/VBD/AVBD mechanisms                                   | Family-specific paper and coupling coverage                                                                                  |
| Collision                | Native World queries and contact generation; standalone collision library capabilities                        | Solver variants can require different CCD/contact primitives; library availability is not integration into every step path   |
| Coupling                 | Shared rigid/articulated constraint solving and bounded rigid-obstacle/deformable interactions                | General bidirectional multiphysics, capability validation and interaction ownership                                          |
| Compute                  | Sequential reference, Taskflow graphs, range execution, SIMD primitives, selected CUDA kernels                | Runtime/kernel comparison, worker/isolation/completion contracts, full-example CUDA execution and later cost-guided grouping |
| State/restart            | Model/state/control components, dense indices, stable serialization IDs, replay and binary snapshots          | Complete rigid state and fresh-process CPU/CUDA continuation, then wider family/domain coverage                              |
| Research evidence        | Resolved configuration, StepMetrics and selected shared-corpus comparisons                                    | Full promised scene/method/backend matrix with independent physical oracles and current-build timings                        |

### Checked Available Entrypoints

These scoped availability markers retain the source/test contract enforced by
`check-architecture-page-lint`. Availability does not close the broader gaps above.

| Contract                               | Status       | Source symbol and DART test                                                                                                                                       |
| -------------------------------------- | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Free rigid authoring                   | ✅ available | `World::addRigidBody`; [World tests](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world.cpp)                                        |
| Articulated authoring                  | ✅ available | `World::addMultibody`; [contact tests](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world_contact_parity.cpp)                       |
| Default rigid selection                | ✅ available | `WorldOptions::rigidBodySolver`; [default-step goldens](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world_default_step_golden.cpp) |
| Default articulated selection          | ✅ available | `WorldOptions::multibodyOptions`; [contact tests](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world_contact_parity.cpp)            |
| Native collision queries               | ✅ available | `dart::collision::native`; [collision tests](https://github.com/dartsim/dart/blob/main/tests/unit/collision/test_collision_world.cpp)                             |
| Baked rigid candidate/contact capacity | ✅ available | `RigidCollisionCapacityOptions`; [World capacity/overflow tests](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world.cpp)            |
| Sphere/capsule time-of-impact queries  | ✅ available | `CollisionGroup::sphereCast`, `CollisionGroup::capsuleCast`; [CCD tests](https://github.com/dartsim/dart/blob/main/tests/unit/collision/test_ccd.cpp)             |
| Sequential graph execution             | ✅ available | `SequentialExecutor`; [graph tests](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/compute/test_compute_graph.cpp)                               |
| Parallel independent graph nodes       | ✅ available | `ParallelExecutor`; [graph tests](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/compute/test_compute_graph.cpp)                                 |

The source-backed [architecture assessment](https://github.com/dartsim/dart/blob/main/docs/design/dart7_architecture_assessment.md)
records precise gaps and owning files. It is updated at packet intake and
acceptance, and across families when storage, scheduling, coupling or
continuation changes.

## Defaults, Explicit Choices And Research Variants

The long-term default is a versioned selection policy over a maintained solver
portfolio. It validates scene requirements before choosing a tested method or
combination. Advanced explicit choices use the same checks and report an error
when unsupported. A small DART-owned device preference is accepted design;
Taskflow/CUDA runtime objects and kernel details remain private. This does not
claim the complete selection policy is implemented today.

Paper implementations can remain versioned variants for reproducibility while
new defaults use better-supported variants. A checkpoint pins what actually
ran, not just the word `auto`. No solver is assumed best for every physical
model, accuracy target or hardware configuration.

## Readiness Sequence

[PLAN-040](https://github.com/dartsim/dart/blob/main/docs/plans/040-dart7-release-hardening.md)
owns the milestone sequence, proposed release cut, backend requirements and
acceptance criteria. Consult that coordinator for the current required
examples and evidence; this architecture page does not maintain a second
release checklist.

Backend claims need actual runtime evidence: GPU compilation or CPU fallback
does not demonstrate CUDA execution. Performance depends on the workload.
DART 6 can provide differential comparisons; independent physical correctness
defines DART 7 acceptance.

## Source-of-truth Map

| Topic                                         | Owner                                                                                                                                                                                                                                                                                       |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Milestones, dependency packets and gaps       | [PLAN-040](https://github.com/dartsim/dart/blob/main/docs/plans/040-dart7-release-hardening.md), [dashboard](https://github.com/dartsim/dart/blob/main/docs/plans/dashboard.md)                                                                                                             |
| Source-backed audit and standing rule         | [Architecture assessment](https://github.com/dartsim/dart/blob/main/docs/design/dart7_architecture_assessment.md)                                                                                                                                                                           |
| Solver portfolio, coupling and engine lessons | [Solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md)                                                                                                                                                                              |
| Public C++ and Python workflows               | [C++ facade](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md), [Python facade](https://github.com/dartsim/dart/blob/main/docs/design/simulation_python_api.md)                                                                                                  |
| Library selection and compute graphs          | [Compute research](https://github.com/dartsim/dart/blob/main/docs/design/compute_backend_research.md), [compute decisions](https://github.com/dartsim/dart/blob/main/docs/design/scalable_compute_decisions.md)                                                                             |
| New methods and paper evidence                | [Solver intake](https://github.com/dartsim/dart/blob/main/docs/plans/solver-family-intake.md), [extension contracts](https://github.com/dartsim/dart/blob/main/docs/design/algorithm_extension_contracts.md)                                                                                |
| Differentiation                               | [Differentiable simulation](https://github.com/dartsim/dart/blob/main/docs/design/differentiable_simulation.md)                                                                                                                                                                             |
| Product direction and releases                | [North star](https://github.com/dartsim/dart/blob/main/docs/ai/north-star.md), [clean-break strategy](https://github.com/dartsim/dart/blob/main/docs/design/dart7_clean_break_strategy.md), [release roadmap](https://github.com/dartsim/dart/blob/main/docs/onboarding/release-roadmap.md) |
