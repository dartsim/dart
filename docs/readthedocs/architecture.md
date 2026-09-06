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

## The Living Architecture Map

The four views below are rendered at documentation build time from typed JSON
sources under `docs/assets/architecture/`. Each node carries a status tag. In
the three architecture views every node cites the source files that back it,
and the `SRC` markers open those files at the commit the site was built from;
the data-flow view's nodes are the step-stage slots themselves, so their
evidence is the checked one-to-one mapping to the schedule enumerators rather
than file links. A CI check fails when a cited path, line, or
symbol disappears, or when a simulation module, step-stage slot, or solver
family is missing from the view that owns it, so the map cannot drift silently
behind the code. Pan, zoom, search, and node focus work inside each frame; the
"Open the interactive view" link gives the full viewer with guided chapters
and exports.

**Status tags** follow the
[architecture assessment](https://github.com/dartsim/dart/blob/main/docs/design/dart7_architecture_assessment.md):
`Implemented` means a source path and a scoped test exist, `Partial` means a
useful implementation with uncovered contract cells, `Planned` means accepted
work without qualifying implementation, and `Undecided` marks an open design
decision. Dashed relationships lead to planned work.

**Node colors** reuse the renderer's fixed type palette with DART meanings:
`external` is the user-facing facade or a third-party library, `frontend` is a
dartpy or dartsim surface, `backend` is solver, stage, or compute code,
`database` is model, state, storage, checkpoint, or replay data, `messagebus`
is coupling, exchange, or the step schedule, `cloud` is an executor or device
runtime, and `security` marks the public-API boundary that hides internals.

### Simulation Framework

Everything a user touches sits inside the public boundary on the left: the
World facade, the scene handles, and the option values that select method
families. Selection resolves to an ordered stage schedule; the schedule drives
the rigid, multibody, and deformable solver families and emits compute graphs.
Coupling beyond shared constraint rows and rigid-obstacle contact is planned.

```{raw} html
<div class="arch-map">
  <iframe src="architecture-map/simulation-framework.html?theme=light&embed=1" title="DART 7 simulation framework map" loading="lazy"></iframe>
  <p class="arch-map__links"><a href="architecture-map/simulation-framework.html?theme=light" target="_blank" rel="noopener">Open the interactive view</a> · source: <code>docs/assets/architecture/simulation-framework.architecture.json</code></p>
</div>
```

### World::step() Data Flow

The built-in schedule is an ordered stage list, not one executable DAG. Which
slots run depends on the selected families: the split rigid pipeline
(sequential impulse, VBD, AVBD) advances velocity, contact, and position as
separate stages; IPC uses one combined contact-and-advance stage; semi-implicit
multibody fuses into the unified constraint solve while variational integration
runs standalone. Every flow names the data a stage hands on, so the owner of
each state between stages is readable. Kinematics always runs last and feeds
diagnostics, replay, and checkpoints.

```{raw} html
<div class="arch-map">
  <iframe src="architecture-map/world-step.html?theme=light&embed=1" title="World::step() data flow" loading="lazy"></iframe>
  <p class="arch-map__links"><a href="architecture-map/world-step.html?theme=light" target="_blank" rel="noopener">Open the interactive view</a> · source: <code>docs/assets/architecture/world-step.dataflow.json</code></p>
</div>
```

### Compute Graph

Three layers stay separate: the semantic graph with explicit edges as the
correctness source of truth, the executable plan that binds work to buffers
and kernels, and the runtime adapters (sequential reference, Taskflow, resident
CUDA kernels) that submit already-defined work. Profiles, metrics, and the DOT
export are the evidence surface. Dependency inference, grouping, and
asynchronous device completion are planned and shown as such.

```{raw} html
<div class="arch-map">
  <iframe src="architecture-map/compute-graph.html?theme=light&embed=1" title="DART 7 compute graph" loading="lazy"></iframe>
  <p class="arch-map__links"><a href="architecture-map/compute-graph.html?theme=light" target="_blank" rel="noopener">Open the interactive view</a> · source: <code>docs/assets/architecture/compute-graph.architecture.json</code></p>
</div>
```

### Library Context

The whole library at a glance: the DART 7 core modules, the classic modules
that DART 6 users know, the dartpy and dartsim surfaces, and the external
dependencies each module owns. Use it to find which module a topic lives in
before opening the developer handbook.

```{raw} html
<div class="arch-map">
  <iframe src="architecture-map/library-context.html?theme=light&embed=1" title="DART library context map" loading="lazy"></iframe>
  <p class="arch-map__links"><a href="architecture-map/library-context.html?theme=light" target="_blank" rel="noopener">Open the interactive view</a> · source: <code>docs/assets/architecture/library-context.architecture.json</code></p>
</div>
```

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
continuation changes; the map views above are updated in the same change.

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
| Architecture map views and their checks       | [`docs/assets/architecture/`](https://github.com/dartsim/dart/tree/main/docs/assets/architecture), `pixi run check-architecture-map`, `pixi run render-architecture-map`; [design decisions](https://github.com/dartsim/dart/blob/main/docs/design/architecture_map.md)                     |
| Milestones, dependency packets and gaps       | [PLAN-040](https://github.com/dartsim/dart/blob/main/docs/plans/040-dart7-release-hardening.md), [dashboard](https://github.com/dartsim/dart/blob/main/docs/plans/dashboard.md)                                                                                                             |
| Source-backed audit and standing rule         | [Architecture assessment](https://github.com/dartsim/dart/blob/main/docs/design/dart7_architecture_assessment.md)                                                                                                                                                                           |
| Solver portfolio, coupling and engine lessons | [Solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md)                                                                                                                                                                              |
| Public C++ and Python workflows               | [C++ facade](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md), [Python facade](https://github.com/dartsim/dart/blob/main/docs/design/simulation_python_api.md)                                                                                                  |
| Library selection and compute graphs          | [Compute research](https://github.com/dartsim/dart/blob/main/docs/design/compute_backend_research.md), [compute decisions](https://github.com/dartsim/dart/blob/main/docs/design/scalable_compute_decisions.md)                                                                             |
| New methods and paper evidence                | [Solver intake](https://github.com/dartsim/dart/blob/main/docs/plans/solver-family-intake.md), [extension contracts](https://github.com/dartsim/dart/blob/main/docs/design/algorithm_extension_contracts.md)                                                                                |
| Differentiation                               | [Differentiable simulation](https://github.com/dartsim/dart/blob/main/docs/design/differentiable_simulation.md)                                                                                                                                                                             |
| Product direction and releases                | [North star](https://github.com/dartsim/dart/blob/main/docs/ai/north-star.md), [clean-break strategy](https://github.com/dartsim/dart/blob/main/docs/design/dart7_clean_break_strategy.md), [release roadmap](https://github.com/dartsim/dart/blob/main/docs/onboarding/release-roadmap.md) |
