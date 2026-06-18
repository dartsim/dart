# DART 7 Architecture: Multi-Physics, Multi-Solver, Multi-Backend

```{note}
This page describes the **DART 7 simulation engine** — the promoted
`dart::simulation::World`, which is the clean-break public API for DART 7. It
is the single-page visual map of how that one pipeline is
generalized so it can support **many physics domains, many solver methods, and
many compute backends** at once. Everything below is DART 7.

The classic DART 6 API (`dart::simulation::World`, `Skeleton`/`BodyNode`/`Joint`,
the FCL/Bullet/ODE collision backends) is **not** part of this engine. It is
maintained separately on `release-6.*` compatibility branches and is out of
scope here; see the
[clean-break strategy](https://github.com/dartsim/dart/blob/main/docs/design/dart7_clean_break_strategy.md).

DART 7 promotion is **parity-gated**: parity claims must come from direct
evidence, with DART 6 comparisons sourced from `release-6.*` branches. Boxes
below are marked **available** (in the DART 7 stack today),
**experimental / opt-in** (active research tracks), or **planned**. Owner
documents and headers are the
source of truth; this page is a navigational snapshot. For live progress and
sequencing see the
[plan dashboard](https://github.com/dartsim/dart/blob/main/docs/plans/dashboard.md)
and the
[release roadmap](https://github.com/dartsim/dart/blob/main/docs/onboarding/release-roadmap.md).
```

## The design in one sentence

> The `World` owns topology, time, and a configured set of **solvers**; each
> solver advances the dynamics of the entities in its **physics domain**, and
> **couplers** mediate interactions between domains — with parallelizable work
> expressed as **compute-graph** nodes that any **backend executor** can run.
> Users configure method families and policies, never solver registries,
> component storage, or execution backends.

Everything else on this page is a consequence of that sentence: each abstracted
box in the pipeline is a seam where DART can offer more than one option, and the
`World` composes the chosen options into one deterministic step.

## Why three axes of choice

DART 7 generalizes the pipeline along three independent axes — physics,
solver/algorithm, and compute backend — for three concrete reasons. These map
directly onto the three [north-star](https://github.com/dartsim/dart/blob/main/docs/ai/north-star.md)
research dimensions.

| Axis              | What it means                                                                         | Who it is for                                                                                                                                                                                |
| ----------------- | ------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Multi-physics** | Rigid, articulated, deformable, and later particle/fluid domains in one coupled step. | Researchers and users who need more than rigid-body dynamics in a single scene.                                                                                                              |
| **Multi-solver**  | More than one method/algorithm family per domain, selected by capability name.        | **Researchers** can plug in a new paper's method and compare it apples-to-apples against DART baselines on shared foundations (collision, math, memory, threading, SIMD, tests, benchmarks). |
| **Multi-backend** | The same solver work runs on a sequential, multi-core, Taskflow, or CUDA executor.    | **End users** pick the option that fits their platform and scene; defaults adapt so the easy path stays easy.                                                                                |

The three motivations behind these axes:

1. **Research, apples-to-apples.** DART is research-focused. A new algorithm
   should be reproducible and benchmarkable _inside_ DART against existing
   baselines, not in a one-off fork. New paper methods enter through DART-owned
   solver _families_ that reuse shared components, so comparisons are fair.
   See [algorithm extension contracts](https://github.com/dartsim/dart/blob/main/docs/design/algorithm_extension_contracts.md)
   and the [solver/multi-physics architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md).
2. **End-user choice.** Different users have different accuracy, speed, and
   platform constraints. Exposing solver method families and (internally)
   backend executors lets a user choose the best option for their problem
   instead of accepting a single hard-wired path.
3. **Auto-configuration.** The common path must stay trivial. The `World`
   selects a sensible default solver per domain from scene content today, and
   the backend seam is designed so platform-aware and scene-scale-aware
   selection (CPU vs. GPU, small vs. large/batched scenes) can be layered in
   without changing the public API.
   See [scalable compute decisions](https://github.com/dartsim/dart/blob/main/docs/design/scalable_compute_decisions.md).

## The simulation pipeline as abstracted boxes

Each box below is an abstraction seam. The label names the responsibility; the
list inside names the **options available at that seam** (with status markers).

```
WORLD  — owns topology · time · frames · the step schedule; picks
         per-domain defaults; validates options; exposes
         method-family names & policies (no solver / backend /
         registry types on the public facade)
                                  │ composes
                                  ▼
PHYSICS DOMAINS — each entity assigned to a solver by its physical model
┌─────────────────────┬─────────────────────┬─────────────────────┐
│ rigid bodies    [A] │ articulated     [A] │ deformable      [X] │
│ particles       [P] │ multibody           │ fluid           [P] │
└─────────────────────┴─────────────────────┴─────────────────────┘
         couplers mediate each domain pair: pairwise, swappable
         strategy (penalty/projection · convex · implicit)   [P]
                                  │
                                  ▼
SOLVERS — one method family advances each domain
  rigid:       sequential-impulse [A] · IPC [X] · boxed-LCP [X]
  multibody:   semi-implicit joint-space [A] · variational [X]
  deformable:  mass-spring · neo-Hookean FEM · projected-Newton · VBD [X]
  diff. grad:  analytic · complementarity-aware · pre-contact [X]
                                  │
                                  ▼
COLLISION / CONTACTS — dart::collision::native [A]
  AABB broad-phase · narrow-phase · contact manifolds ·
  swept/CCD casts [A] · persistent manifold cache · SDF
  (library capability; World contact-path integration [P])
  → typed contact buffers consumed by solvers / couplers
                                  │
                                  ▼
COMPUTE GRAPH — solver/coupler work = nodes + explicit deps
  kinematics + free-rigid integration run as graphs [A];
  graph execution of the remaining stages [P] (today they
  run inside the ordered stage schedule) · stage metadata ·
  profiling · DOT visualization ·
  WorldStepPipeline / WorldStepStage composition seams [A]
                                  │
                                  ▼
COMPUTE BACKEND — injected through the ComputeExecutor seam
  sequential (reference, default)                [A]
  parallel — Taskflow-backed multi-core CPU      [A]
  CUDA / GPU — opt-in sidecar, CPU fallback      [X]
  SIMD multi-ISA foundation (SSE…AVX-512 / NEON)
  — library available; simulation-pipeline use   [P]

Status:  [A] available in the DART 7 stack today  ·
         [X] experimental / opt-in  ·  [P] planned
```

### Step schedule: current and planned

**Today** one `World::step()` runs a flat, content-aware ordered stage
schedule (owned internally by `detail/world_step_schedule.hpp`): the `World`
enters simulation mode (freezing topology and preparing each active stage),
emits only the stage slots whose domains have entities, executes them in
order, then advances time/frame counters and refreshes kinematics for fresh
reads. There are no substeps and no coupling phases in the current schedule.

**Planned** [P] — when cross-domain couplers land, the schedule generalizes
to substep windowing so heterogeneous solvers can interact without knowing
about each other. For a single-domain world with no coupling the `couple`
phase is empty and the schedule collapses to today's plain ordered step — no
overhead for the common case.

```
World::step()                                            (planned [P] shape)
  └─ enter simulation mode (freeze topology, finalize each active solver)
  └─ for each substep:
        refresh collision / contact generation
        ┌─────────┐   ┌────────────┐   ┌─────────┐   ┌──────────────┐
        │ prepare │ → │ pre-couple │ → │ couple  │ → │ post-couple  │ → integrate
        └─────────┘   └────────────┘   └─────────┘   └──────────────┘
          each active     advance to     couplers      complete the
          solver readies  the coupling   exchange      substep with
          inputs          boundary       cross-domain  coupled state
                                         interaction
  └─ advance time & frame counters · refresh kinematics for fresh reads
```

`World::step()` stays synchronous, deterministic, and complete on return.
Sequential execution is the reference path; other executors must match it.

## Options catalog

The seams above, with the concrete options that exist today, how they are
selected from the public facade, and the owner document for details. Header and
owner docs are authoritative; this table is a snapshot.

Throughout, **✅ available** means "present and selectable in the DART 7 stack
today." **🧪 experimental** is an opt-in active research track, and **📋 planned**
has an agreed design but no implementation yet.

### Physics domains

| Domain                | Status          | Public entry point                       | Owner                                                                                                                                                                                                                                                            |
| --------------------- | --------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Rigid bodies          | ✅ available    | `World::addRigidBody`, rigid-body joints | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md); tested by [`test_world.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world.cpp)                               |
| Articulated multibody | ✅ available    | `World::addMultibody`                    | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md); tested by [`test_world_contact_parity.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world_contact_parity.cpp) |
| Deformable bodies     | 🧪 experimental | `World::addDeformableBody`               | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md)                                                                                                                                                   |
| Particles / fluids    | 📋 planned      | —                                        | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md)                                                                                                                                                   |

Entities are assigned to a solver by **physical model**, not geometry, so the
same shape is usable across domains and `World::add*` stays uniform.

### Solver method families

| Domain     | Method option                                                                       | Status          | Selected by                                                                                                                                                                                                                     |
| ---------- | ----------------------------------------------------------------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Rigid      | Sequential-impulse (default)                                                        | ✅ available    | `WorldOptions::rigidBodySolver` or `World::setRigidBodySolver(...)`; tested by [`test_world_default_step_golden.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world_default_step_golden.cpp) |
| Rigid      | IPC (incremental potential contact)                                                 | 🧪 experimental | `WorldOptions::rigidBodySolver` or `World::setRigidBodySolver(...)`                                                                                                                                                             |
| Rigid      | Contact normal/friction: sequential-impulse / boxed-LCP                             | 🧪 experimental | `WorldOptions::contactSolverMethod`                                                                                                                                                                                             |
| Rigid      | Differentiable gradient: analytic / complementarity-aware / pre-contact surrogate   | 🧪 experimental | `WorldOptions::differentiable`, `WorldOptions::contactGradientMode`, setter                                                                                                                                                     |
| Multibody  | Semi-implicit joint-space integration (default)                                     | ✅ available    | `WorldOptions::multibodyOptions` or `World::setMultibodyOptions(...)`; tested by [`test_world_contact_parity.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/world/test_world_contact_parity.cpp)         |
| Multibody  | Variational integrator (discrete-mechanics; linear-time form is PLAN-084, proposed) | 🧪 experimental | `WorldOptions::multibodyOptions` or `World::setMultibodyOptions(...)`                                                                                                                                                           |
| Deformable | Mass-spring (default) / stable neo-Hookean FEM (opt-in)                             | 🧪 experimental | `DeformableBodyOptions`                                                                                                                                                                                                         |
| Deformable | Projected-Newton + self-contact barrier / friction; VBD block descent               | 🧪 experimental | `World::configureDeformableSolver`                                                                                                                                                                                              |

New paper methods enter through the nearest DART-owned family (the IPC family
for deformable, rigid, and affine/unified IPC variants consolidated through the
unified Newton-barrier implementation; VBD/AVBD under the VBD family;
differentiable LCP under the differentiable rigid family) so they share a
domain, state adapter, contact representation, benchmark schema, and capability
vocabulary instead of forming isolated stacks. Solvers, presets, and examples
use method/approach/paper or DART-owned names — never other engines' names.

### Cross-domain coupling

| Concept                                    | Status                            | Notes                                                                                                                                                                                                                                       |
| ------------------------------------------ | --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Coupler` (pairwise, keyed by domain pair) | 📋 planned (architecture defined) | Coupling method is a swappable strategy (penalty/projection, convex contact, implicit potential) chosen by policy. A solver never branches on which coupler is active. Rigid-internal contact is the rigid solver's own job, not a coupler. |

### Collision and contacts

DART 7 uses one collision system — the native `dart::collision::native`
collision world — reached through `World::collide()` and the internal contact
generation that feeds the solvers. It is not a multi-backend choice: the classic
DART 6 FCL / Bullet / ODE backends are **not** part of the DART 7 pipeline.

| Capability                                                  | Status          | Notes                                                                                                                                                                                                   |
| ----------------------------------------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Native collision world (`dart::collision::native`)          | ✅ available    | AABB broad-phase + narrow-phase, contact manifolds, collision filters. Tested by [`test_collision_world.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/collision/test_collision_world.cpp). |
| Persistent manifold cache, signed-distance-field (SDF) path | 📋 planned      | Implemented in the collision library, but not yet consumed by the DART 7 `World` contact path; pipeline integration is planned.                                                                         |
| Swept / continuous (CCD) sphere & capsule casts             | ✅ available    | Time-of-impact queries via `CollisionGroup::sphereCast` / `CollisionGroup::capsuleCast`. Tested by [`test_ccd.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/collision/test_ccd.cpp).       |
| Typed contact buffers (`contacts` views)                    | 🧪 experimental | Consumed by solvers and couplers; public contact views deferred.                                                                                                                                        |

### Compute backends (executor seam)

| Backend / executor   | Status                 | Notes                                                                                                                                                                                                                                                                                                                                                                                                                            |
| -------------------- | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SequentialExecutor` | ✅ available           | Reference path; defines deterministic semantics. Tested by [`test_compute_graph.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/compute/test_compute_graph.cpp).                                                                                                                                                                                                                                           |
| `ParallelExecutor`   | ✅ available           | Taskflow-backed multi-core CPU; independent compute-graph nodes run concurrently. Today only the kinematics and free-rigid integration stages emit multi-node graphs; dynamics/contact stages run sequentially within the ordered schedule. (`TaskflowExecutor` is a compatibility alias.) Tested by [`test_compute_graph.cpp`](https://github.com/dartsim/dart/blob/main/tests/unit/simulation/compute/test_compute_graph.cpp). |
| CUDA / GPU           | 🧪 experimental opt-in | Sidecar packaging; never a default dependency; requires an identical-semantics CPU fallback. Validated go/no-go (see compute decisions).                                                                                                                                                                                                                                                                                         |
| SIMD (foundation)    | 📋 planned             | Multi-ISA (SSE4.2 … AVX-512, ARM NEON) batch math library exists under `dart/simd/`, but it is not yet consumed by the simulation pipeline, so it remains planned as a pipeline backend.                                                                                                                                                                                                                                         |

The executor is injected through the abstract `compute::ComputeExecutor`
boundary — the only public concurrency seam. No `entt`, thread-pool, GPU device,
stream, kernel, memory-pool, or solver-registry type appears in the public API.
Backend names may appear in build flags, diagnostics, and benchmark reports, but
not in public types, namespaces, or required configuration. See
[scalable compute decisions](https://github.com/dartsim/dart/blob/main/docs/design/scalable_compute_decisions.md),
[compute backend research](https://github.com/dartsim/dart/blob/main/docs/design/compute_backend_research.md),
and [shared CUDA device substrate](https://github.com/dartsim/dart/blob/main/docs/design/shared_cuda_device_substrate.md).

## How configuration stays simple

Multi-everything must not make the common path hard. The configuration contract:

- **Default selection from content.** A free-rigid world gets the
  sequential-impulse path, an articulated multibody world gets the
  semi-implicit joint-space path, and deformable bodies get the deformable
  dynamics path. The built-in schedule emits only the domains that are present,
  so the easy path remains `World` → `addRigidBody`/`addMultibody`/
  `addDeformableBody` → `step` with no solver vocabulary.
- **Method-family names, not engine or backend names.** Advanced users request a
  capability (e.g. `"variational integrator"`, IPC, boxed-LCP) or set a policy.
  The `World` maps it to an internal solver or returns an actionable
  unsupported-capability error.
- **Construction-time grouping.** `WorldOptions` carries initial domain solver
  choices and policies, while post-construction properties/setters remain for
  interactive workflows. This keeps defaults, bindings, and schedule
  preparation on one validated path. Result-affecting World-level solver
  choices round-trip through binary save/load and replay so restarts do not
  silently fall back to default families.
- **No backend leakage.** Backend, ECS storage, registry, and execution types
  stay internal. Switching or adding a backend preserves the public API.
- **Deterministic by default.** `World::step()` is synchronous and reproducible;
  batched/async execution, if added, wraps this contract rather than replacing
  it.

## Model / State / Control / Contacts separation

Underneath the facade, four concerns are kept separate so that batched worlds
(`n_envs`), rollouts, and differentiable simulation become possible without a
rewrite — while `World::step()` keeps the separation hidden on the easy path:

- **Model** — static topology and parameters (frozen at finalization).
- **State** — dynamic per-step values (positions, velocities, caches).
- **Control** — user inputs (targets, efforts, applied loads).
- **Contacts** — typed buffers from collision generation.

## Source-of-truth map

This page is a synthesis. Each detailed rule has one owner:

| Topic                                                          | Owner document                                                                                                                                                                       |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Mission and the three research dimensions                      | [north-star](https://github.com/dartsim/dart/blob/main/docs/ai/north-star.md)                                                                                                        |
| Verified architecture findings and standing rules              | [dart7_architecture_assessment](https://github.com/dartsim/dart/blob/main/docs/design/dart7_architecture_assessment.md)                                                              |
| Solver abstraction, domain assignment, coupling, step schedule | [simulation_solver_architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md)                                                            |
| Public C++ object model and promotion rules                    | [simulation_cpp_api](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md)                                                                                    |
| dartpy surface                                                 | [simulation_python_api](https://github.com/dartsim/dart/blob/main/docs/design/simulation_python_api.md)                                                                              |
| Research extension and baseline contracts                      | [algorithm_extension_contracts](https://github.com/dartsim/dart/blob/main/docs/design/algorithm_extension_contracts.md)                                                              |
| CPU / SIMD / GPU decision framework                            | [scalable_compute_decisions](https://github.com/dartsim/dart/blob/main/docs/design/scalable_compute_decisions.md)                                                                    |
| Backend evidence survey                                        | [compute_backend_research](https://github.com/dartsim/dart/blob/main/docs/design/compute_backend_research.md)                                                                        |
| Differentiable simulation                                      | [differentiable_simulation](https://github.com/dartsim/dart/blob/main/docs/design/differentiable_simulation.md)                                                                      |
| DART 7 vs DART 6 release topology (clean break)                | [dart7_clean_break_strategy](https://github.com/dartsim/dart/blob/main/docs/design/dart7_clean_break_strategy.md)                                                                    |
| Live progress, sequencing, and parity gates                    | [plan dashboard](https://github.com/dartsim/dart/blob/main/docs/plans/dashboard.md), [release roadmap](https://github.com/dartsim/dart/blob/main/docs/onboarding/release-roadmap.md) |

For active sequencing of the work behind these boxes, see the
[living plans](https://github.com/dartsim/dart/blob/main/docs/plans/README.md).
