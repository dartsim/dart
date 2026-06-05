# DART 7 Architecture: Multi-Physics, Multi-Solver, Multi-Backend

```{note}
This page describes the **DART 7 simulation engine** — the experimental
`dart::simulation::experimental` `World`, which is the clean-break public API
target for DART 7. It is the single-page visual map of how that one pipeline is
generalized so it can support **many physics domains, many solver methods, and
many compute backends** at once. Everything below is DART 7.

The classic DART 6 API (`dart::simulation::World`, `Skeleton`/`BodyNode`/`Joint`,
the FCL/Bullet/ODE collision backends) is **not** part of this engine. It is
maintained separately on the `release-6.17` compatibility line and is out of
scope here; see the
[clean-break strategy](https://github.com/dartsim/dart/blob/main/docs/design/dart7_clean_break_strategy.md).

DART 7 promotion is **parity-gated**: the experimental World becomes the promoted
public API only after core robotics workflows have direct parity evidence, so
this is a maturing target, not a finished one. Boxes below are marked
**available** (in the experimental stack today), **experimental / opt-in**
(active research tracks), or **planned**. Owner documents and headers are the
source of truth; this page is a navigational snapshot. For live progress and
sequencing see the
[plan dashboard](https://github.com/dartsim/dart/blob/main/docs/plans/dashboard.md)
and the
[release roadmap](https://github.com/dartsim/dart/blob/main/docs/onboarding/release-roadmap.md).
```

## The design in one sentence

> The `World` owns topology, time, and a configured set of **solvers**; each
> solver advances the dynamics of the entities in its **physics domain**, and
> **couplers** mediate interactions between domains — all expressed as
> **compute-graph** work that any **backend executor** can run. Users configure
> method families and policies, never solver registries, component storage, or
> execution backends.

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
  multibody:   semi-implicit ABA [A] · variational integrator [X]
  deformable:  mass-spring · neo-Hookean FEM · projected-Newton · VBD [X]
  diff. grad:  analytic [A] · complementarity-aware · pre-contact [X]
                                  │
                                  ▼
COLLISION / CONTACTS — dart::collision::native [A]
  AABB broad-phase · narrow-phase · contact manifolds ·
  persistent manifold cache · SDF · swept/CCD  →
  typed contact buffers consumed by solvers / couplers
                                  │
                                  ▼
COMPUTE GRAPH — solver/coupler work = nodes + explicit deps [A]
  stage metadata · profiling · DOT visualization ·
  WorldStepPipeline / WorldStepStage composition seams
                                  │
                                  ▼
COMPUTE BACKEND — injected through the ComputeExecutor seam
  sequential (reference, default)                [A]
  parallel — Taskflow-backed multi-core CPU      [A]
  CUDA / GPU — opt-in sidecar, CPU fallback      [X]
  SIMD multi-ISA foundation (SSE…AVX-512 / NEON) [A]

Status:  [A] available in the experimental stack today, parity-gated
         before DART 7 promotion  ·  [X] experimental / opt-in  ·  [P] planned
```

### Per-substep step schedule

Within one `World::step()` the schedule below runs for each substep. For a
single-domain world with no cross-domain coupling the `couple` phase is empty
and the schedule collapses to a plain solver step — no overhead for the common
case.

```
World::step()
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

Throughout, **✅ available** means "present and selectable in the experimental
stack today" — still parity-gated before DART 7 promotion, not a shipped/promoted
guarantee. **🧪 experimental** is an opt-in active research track, and
**📋 planned** has an agreed design but no implementation yet.

### Physics domains

| Domain                | Status          | Public entry point                       | Owner                                                                                                          |
| --------------------- | --------------- | ---------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Rigid bodies          | ✅ available    | `World::addRigidBody`, rigid-body joints | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md) |
| Articulated multibody | ✅ available    | `World::addMultibody`                    | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md) |
| Deformable bodies     | 🧪 experimental | `World::addDeformableBody`               | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md) |
| Particles / fluids    | 📋 planned      | —                                        | [solver architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md) |

Entities are assigned to a solver by **physical model**, not geometry, so the
same shape is usable across domains and `World::add*` stays uniform.

### Solver method families

| Domain     | Method option                                                                       | Status          | Selected by                                                      |
| ---------- | ----------------------------------------------------------------------------------- | --------------- | ---------------------------------------------------------------- |
| Rigid      | Sequential-impulse (default)                                                        | ✅ available    | `World::setRigidBodySolver(SequentialImpulse)`                   |
| Rigid      | IPC (incremental potential contact)                                                 | 🧪 experimental | `World::setRigidBodySolver(Ipc)`                                 |
| Rigid      | Contact normal: sequential-impulse / boxed-LCP                                      | 🧪 experimental | `WorldOptions::contactSolverMethod`                              |
| Rigid      | Differentiable gradient: analytic / complementarity-aware / pre-contact surrogate   | 🧪 experimental | `WorldOptions::differentiable` + `World::setContactGradientMode` |
| Multibody  | Semi-implicit articulated-body forward dynamics (default)                           | ✅ available    | `MultibodyOptions::integrationFamily = "semi-implicit"`          |
| Multibody  | Variational integrator (discrete-mechanics; linear-time form is PLAN-082, proposed) | 🧪 experimental | `MultibodyOptions::integrationFamily = "variational integrator"` |
| Deformable | Mass-spring (default) / stable neo-Hookean FEM (opt-in)                             | 🧪 experimental | `DeformableBodyOptions`                                          |
| Deformable | Projected-Newton + self-contact barrier / friction; VBD block descent               | 🧪 experimental | `World::configureDeformableSolver`                               |

New paper methods enter through the nearest DART-owned family (rigid IPC and
deformable IPC under the Newton-barrier family; VBD/AVBD under the VBD family;
differentiable LCP under the differentiable rigid family) so they share a domain,
state adapter, contact representation, benchmark schema, and capability
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

| Capability                                                  | Status          | Notes                                                                  |
| ----------------------------------------------------------- | --------------- | ---------------------------------------------------------------------- |
| Native collision world (`dart::collision::native`)          | ✅ available    | AABB broad-phase + narrow-phase, contact manifolds, collision filters. |
| Persistent manifold cache, signed-distance-field (SDF) path | ✅ available    | Stable contacts across steps; SDF-based queries.                       |
| Swept / continuous (CCD) sphere & capsule casts             | ✅ available    | Time-of-impact queries.                                                |
| Typed contact buffers (`contacts` views)                    | 🧪 experimental | Consumed by solvers and couplers; public contact views deferred.       |

### Compute backends (executor seam)

| Backend / executor   | Status                 | Notes                                                                                                                                    |
| -------------------- | ---------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `SequentialExecutor` | ✅ available           | Reference path; defines deterministic semantics.                                                                                         |
| `ParallelExecutor`   | ✅ available           | Taskflow-backed multi-core CPU; independent compute-graph nodes run concurrently. (`TaskflowExecutor` is a compatibility alias for it.)  |
| CUDA / GPU           | 🧪 experimental opt-in | Sidecar packaging; never a default dependency; requires an identical-semantics CPU fallback. Validated go/no-go (see compute decisions). |
| SIMD (foundation)    | ✅ available           | Multi-ISA (SSE4.2 … AVX-512, ARM NEON) batch math under `dart/simd/`.                                                                    |

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

- **Default selection from content.** A rigid/articulated-only world gets the
  articulated-body forward-dynamics + LCP-contacts default — DART 7's own native
  implementation, gated to reach parity with the DART 6 rigid-body capability set
  — with no solver vocabulary. The easy path is
  `World` → `addRigidBody`/`addMultibody` → `step`.
- **Method-family names, not engine or backend names.** Advanced users request a
  capability (e.g. `"variational integrator"`, IPC, boxed-LCP) or set a policy.
  The `World` maps it to an internal solver or returns an actionable
  unsupported-capability error.
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
| Solver abstraction, domain assignment, coupling, step schedule | [simulation_solver_architecture](https://github.com/dartsim/dart/blob/main/docs/design/simulation_solver_architecture.md)                                                            |
| Public C++ object model and promotion rules                    | [simulation_experimental_cpp_api](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_cpp_api.md)                                                          |
| dartpy surface                                                 | [simulation_experimental_python_api](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_python_api.md)                                                    |
| Research extension and baseline contracts                      | [algorithm_extension_contracts](https://github.com/dartsim/dart/blob/main/docs/design/algorithm_extension_contracts.md)                                                              |
| CPU / SIMD / GPU decision framework                            | [scalable_compute_decisions](https://github.com/dartsim/dart/blob/main/docs/design/scalable_compute_decisions.md)                                                                    |
| Backend evidence survey                                        | [compute_backend_research](https://github.com/dartsim/dart/blob/main/docs/design/compute_backend_research.md)                                                                        |
| Differentiable simulation                                      | [differentiable_simulation](https://github.com/dartsim/dart/blob/main/docs/design/differentiable_simulation.md)                                                                      |
| DART 7 vs DART 6 release topology (clean break)                | [dart7_clean_break_strategy](https://github.com/dartsim/dart/blob/main/docs/design/dart7_clean_break_strategy.md)                                                                    |
| Live progress, sequencing, and parity gates                    | [plan dashboard](https://github.com/dartsim/dart/blob/main/docs/plans/dashboard.md), [release roadmap](https://github.com/dartsim/dart/blob/main/docs/onboarding/release-roadmap.md) |

For active sequencing of the work behind these boxes, see the
[living plans](https://github.com/dartsim/dart/blob/main/docs/plans/README.md).
