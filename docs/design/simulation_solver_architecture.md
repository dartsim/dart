# Simulation Solver And Multi-Physics Architecture

## Status

Proposal. This document owns the durable architecture rationale for how the
experimental simulation `World` runs physics: the solver abstraction, how
entities are assigned to solvers, how multiple physics domains are coupled, how
static model data is separated from dynamic state, and how all of this maps onto
the existing compute-graph executor.

It is the internal-organization companion to the public-facade design docs:

- [`simulation_experimental_cpp_api.md`](simulation_experimental_cpp_api.md)
  owns the public C++ object model, naming policy, and DART 7 promotion rules.
- [`simulation_experimental_python_api.md`](simulation_experimental_python_api.md)
  owns the dartpy surface.

This doc does not restate those rules; it explains the engine architecture that
sits behind that facade. Active phase tracking lives in
`docs/plans/dashboard.md`; current solver-family owners include PLAN-080
(rigid/articulated baseline), PLAN-081/082/083 (implicit-barrier and unified
Newton-barrier work), PLAN-104 (VBD/AVBD), and PLAN-110 (differentiable
simulation).

## Purpose

The experimental `World` must grow from its current state (a single hard-wired
free-body integration stage plus forward kinematics) into a simulator that:

1. runs full rigid-body dynamics with constraints and contacts, matching and
   then improving on the legacy DART 6 capability set;
2. supports more than one solver method for the same domain (for example an
   articulated-body forward-dynamics solver and, later, a position-based or
   implicit time-stepping solver) without forking the user-facing object model;
3. supports more than one physics domain (rigid bodies first; deformables,
   particles, and fluids later) coupled inside one deterministic step;
4. keeps a stable public facade so the solver internals, execution backend, and
   data layout can change before and after DART 7 promotion.

The core design sentence is:

> The `World` owns topology, time, and a configured set of solvers; each solver
> advances the dynamics of the entities in its physics domain, and couplers
> mediate interactions between domains. Users configure method families and
> policies, never solver registries, component storage, or execution backends.

## Design Principles

These extend the experimental-API principles; they do not replace them.

### Solvers Own Dynamics, The World Owns Composition

The `World` is the owner of topology, names, frames, time, and the step
schedule. It does not itself implement dynamics. A `Solver` implements the
dynamics for one physics domain over the entities assigned to it. The `World`
composes one or more solvers plus couplers into a deterministic step. This keeps
`World` small and lets dynamics methods evolve independently.

### Domain-Driven Assignment, Not Geometry-Driven

An entity is assigned to a solver by its **physics domain** — the kind of
physical model it obeys (rigid body, articulated rigid body, deformable,
particle/continuum, fluid) — not by its geometry or its render shape. Geometry
describes collision and appearance; the physical model decides which solver
integrates the entity. This separation keeps the same shape usable across
solvers and keeps a uniform `World::add*` surface.

### Method Names, Not Engine Names

A solver is documented by capability: integration family, dynamics approach,
constraint solve, coordinate support, supported features, execution shape, and
differentiability (the capability matrix in
[`simulation_experimental_cpp_api.md`](simulation_experimental_cpp_api.md)).
Public selection uses method/approach/paper or DART-owned names — for example
`articulated-body`, `semi-implicit`, `projected Gauss-Seidel`, `XPBD`,
`implicit time stepping`. Solver, preset, and example names must not be derived
from other engines or projects.

### Paper Methods Enter Through Families, Not Silos

DART is research-focused, so adding algorithms from new papers is expected.
That does not mean each paper gets an isolated solver stack. A new paper method
must first be routed to the nearest DART-owned family: for example deformable
IPC, rigid IPC, and affine/unified IPC belong under the Newton-barrier family;
VBD, OGC, and AVBD belong under the VBD family; differentiable LCP and Dojo-style
variational gradients belong under the differentiable/variational rigid-body
families. A new family is justified only when the method cannot share a domain,
state adapter, objective, contact representation, optimization loop, benchmark
schema, or public capability vocabulary with an existing family.

DART 7 is a clean-break API line, so backward compatibility is not a reason to
carry duplicate public solver surfaces. Prefer a clean internal contract and a
single public capability vocabulary over preserving an early experimental shape.

### Shared Components Are The Default

When two solver families need the same concept, the second use should trigger a
promotion decision instead of a copy. Shared candidates include collision and
contact candidate generation, primitive distances, barrier/friction kernels,
kinematics and state adapters, line-search contracts, PSD projection, sparse
linear solves, projected-Newton diagnostics, benchmark JSON schemas, py-demo
scene plumbing, and compute-backend gates. A variant may keep local code while a
contract is unstable, but its plan or dev-task handoff must state why the code
cannot consume an existing shared component and what evidence would allow
promotion later.

Shared does not mean lowest-common-denominator. A promoted component must keep
variant-specific correctness oracles and expose extension points for legitimate
differences such as endpoint-linear versus curved rigid CCD, generalized versus
affine coordinates, or block descent versus Newton solves.

### Coupling Is A First-Class, Pairwise, Swappable Strategy

Interaction between two physics domains is a `Coupler` concept, not ad-hoc code
inside each solver. Couplers are pairwise (keyed by an unordered domain pair) so
domains can be added without editing existing solvers, and the coupling method
is a swappable strategy (penalty/projection, convex contact, implicit
potential) chosen by policy. A solver must never branch on which coupler is
active; the coupler reads and writes exposed intermediate state through a stable
seam.

### Substep Windowing For Multi-Physics

A single `World::step()` may run one or more substeps. Within a substep, solvers
expose intermediate state at defined points so couplers can act between them. The
reference schedule per substep is:

```
prepare        # each active solver readies inputs for this substep
pre-couple     # each active solver advances to the coupling boundary
couple         # registered couplers exchange/resolve cross-domain interaction
post-couple    # each active solver completes the substep using coupled state
```

For a rigid-body-only world with no cross-domain coupling, the `couple` phase is
empty and the schedule collapses to a plain solver step — there is no overhead
or behavior change for the common single-domain case.

### Model / State / Control / Contacts Separation

The long-term data architecture separates four concerns, already named in the
public-facade doc and made load-bearing here:

- **Model**: static topology and parameters (links, joints, inertias, limits,
  geometry, solver parameters). Set up in design mode; frozen at finalization.
- **State**: dynamic values that change every step (positions, velocities,
  accelerations, time, solver work values, caches).
- **Control**: user inputs (joint targets, efforts, applied forces/impulses).
- **Contacts**: typed buffers produced by collision generation and consumed by
  solvers and couplers.

This separation is what makes rollouts, replicated/batched worlds, and
differentiable simulation possible later (one model, many states). The current
implementation stores all four in the ECS registry; the public facade exposes
handles plus state/control/contact views with documented ownership, never the
registry. The separation is an architectural contract, not a requirement to
expose four objects to common users — `World::step()` hides it on the easy path.

### Deterministic Sync Is The Reference

`World::step()` stays synchronous, deterministic, and complete on return, with
fresh outputs for the stages it executed. Substepping, solver iteration, and
coupling all run inside that synchronous contract. Asynchronous/batched
execution, if added, wraps this contract; it does not replace it.

## Core Concepts

| Concept         | Responsibility                                                                                 | Public exposure (DART 7)                         |
| --------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `World`         | Owns topology, time, frames, the configured solver set, couplers, and the step schedule.       | Public facade (existing).                        |
| `PhysicsDomain` | Tag identifying the physical model of an entity (rigid body, articulated, deformable, ...).    | Internal tag; surfaces only as documented enums. |
| `Solver`        | Advances dynamics for one domain over its assigned entities; emits compute work; reports caps. | Not a public type yet; selected by method name.  |
| `Coupler`       | Resolves interaction between a pair of domains during the couple phase.                        | Not public yet; selected by policy.              |
| `Model`         | Static topology + parameters owner.                                                            | Hidden behind handles/value objects.             |
| `State`         | Dynamic per-step values.                                                                       | State views / explicit copy-write-back.          |
| `Control`       | User targets, efforts, applied loads.                                                          | Control views / actuator handles.                |
| `Contacts`      | Typed contact/constraint buffers.                                                              | Contact views (deferred).                        |
| Step schedule   | Ordered substep windowing (prepare / pre-couple / couple / post-couple).                       | Hidden; pipeline/stage overloads for advanced.   |

These map onto the existing implementation primitives: the ECS registry holds
Model/State/Control/Contacts components; the compute graph and executors run the
solver and coupler work; `WorldStepPipeline`/`WorldStepStage` are the current
seam that the solver schedule generalizes.

## Solver Interface

A solver is an internal object with a stable contract. The public API never
hands out the solver type in DART 7; the `World` constructs and owns solvers
based on configuration and content. The conceptual contract is:

- **capabilities()** — the capability matrix values this solver supports, used
  for default selection, validation, and unsupported-feature errors.
- **domain()** — the physics domain it integrates.
- **finalize(model)** — allocate per-solver runtime state once topology is
  frozen; the analogue of the design-mode → simulation-mode transition.
- **substep work** — emit compute-graph nodes for the prepare / pre-couple /
  post-couple phases so the executor runs them; a single-domain solver may emit
  one fused node.
- **state/control/contact access** — expose the views a coupler or the public
  facade needs, without exposing storage.

Solvers must not own world-global concerns (time advance, frame counters,
topology). Those stay on the `World`. Solvers must not branch on coupler
identity. Multiple solvers for the same domain are allowed; only one integrates
a given entity per step.

## Coupling Interface

A coupler resolves the interaction between two domains. Its contract:

- **pair()** — the unordered domain pair it handles.
- **prepare(states)** — optional pre-pass before solvers reach the boundary.
- **couple(states)** — read exposed intermediate state from both domains and
  apply the cross-domain resolution (impulses, projected velocities, shared
  constraint rows).

Couplers are registered against domain pairs so a new domain adds couplers
without touching existing solvers. The coupling method (penalty/projection,
convex contact, implicit) is a strategy choice recorded as policy and reported
in diagnostics. Rigid-internal contact and constraint solving is the
rigid-body solver's own responsibility, not a coupler; couplers handle
_cross-domain_ interaction. A coupler reverse pass exists for differentiability
of cross-domain interaction when that capability is added; the rigid-body
solver's own contact/dynamics gradients are a separate _solver-internal_ reverse
pass (see
[`differentiable_simulation.md`](differentiable_simulation.md)), not a coupler
concern.

## Step Lifecycle

Design mode builds topology and parameters (Model). The first `World::step()`
(or an explicit finalize) freezes topology and calls `finalize(model)` on each
active solver, allocating State/Control/Contacts runtime storage. Each
`World::step()` then runs, for `substeps` iterations:

1. refresh collision/contact generation for the substep (when contacts are
   enabled);
2. run the prepare / pre-couple / couple / post-couple schedule across active
   solvers and registered couplers via the compute graph;
3. integrate state and advance internal substep accounting.

After the substep loop, the `World` advances time and frame counters once per
`step()`, refreshes kinematics for fresh reads, and clears per-step control
accumulators following a documented policy. Active solvers are those that have
at least one assigned entity; an empty domain contributes nothing.

This generalizes the current pipeline: today `World::step()` runs a
`RigidBodyIntegrationStage` then a `KinematicsStage`. Under this architecture
the rigid-body work becomes the rigid-body solver's substep contribution, and
kinematics refresh remains a world-level post-step concern.

## Default Solver Selection

Common users should not choose a solver. The `World` selects a sensible default
per domain from content:

| Content                         | Default method family                                                         |
| ------------------------------- | ----------------------------------------------------------------------------- |
| Rigid bodies / articulated only | Articulated-body forward dynamics + LCP contacts (the DART 6-equivalent path) |
| Deformables (later)             | Position-based / implicit deformable solver                                   |
| Particles / fluids (later)      | Continuum/particle solver                                                     |
| Mixed domains (later)           | Per-domain defaults plus registered couplers                                  |

Advanced users may request a method family or set policies; the `World` returns
documented fallback behavior or an unsupported-capability error when the build
lacks the implementation. Solver registries, plugin loaders, and accelerator
resource handles are not part of this contract and require their own design.

## Configuration Surface Rules

Solver configuration starts from the user's `World` creation path, not from
internal solver objects. The common path should be:

- create a `World` with a small options object or default constructor;
- add bodies/domains through the public facade;
- set at most a method-family or policy preset when the default is not desired;
- call `step()` and read diagnostics when a requested capability is unsupported.

The configuration design must keep simple use simple and invalid use hard:

- use typed options, named policies, validated units, bounded numeric ranges,
  and documented defaults instead of loosely related scalar fields;
- validate incompatible options before or during finalization with actionable
  unsupported-capability errors;
- group advanced algorithm knobs under method-specific nested option objects so
  common `World` setup is not flooded by every paper parameter;
- keep option names DART-owned and capability-oriented, while allowing
  provenance in internal tests, manifests, and benchmark rows;
- preserve serialization/restart behavior for options that affect simulation
  results, or document why an option is runtime-only;
- expose diagnostics that explain which method family, fallback, or unsupported
  path was used without exposing solver registries, ECS storage, or backend
  resources.

Advanced options should scale with the algorithms DART implements, but they
should not require users to understand storage layout, stage order, reference
project terminology, or compute-backend internals. If an advanced option is
only meaningful for one method family, keep it scoped to that method's options
object; do not add a broad `WorldOptions` field that is invalid for most
worlds.

## Solver Family Intake Gate

Before starting a new solver, algorithm, paper, or major component
implementation, the owner plan or dev task must answer this intake checklist:

1. **Family routing.** Name the existing DART solver family and owner plan that
   receives the work, or justify a new family. Examples: PLAN-081/082/083 for
   IPC and Newton-barrier variants, PLAN-104 for VBD/OGC/AVBD variants,
   PLAN-110 plus PLAN-082 for differentiable variational rigid-body work.
2. **Shared-component inventory.** List the collision, kinematics, model/state,
   contact buffer, numeric optimization, linear-solver, compute-backend,
   diagnostics, benchmark, and example components the slice will reuse. Any new
   duplicate must name the missing contract that prevents reuse.
3. **Promotion trigger.** Define what second-use evidence would move a
   variant-local piece into a shared internal owner, and what tests will prove
   old and new consumers still agree.
4. **Apples-to-apples evidence.** Define the DART incumbent, reference
   implementation, paper number, scene corpus, accuracy metric, and benchmark
   JSON shape used for comparison. A performance claim without matched accuracy
   and matched scene parameters is not a completion claim.
5. **Public boundary.** Confirm public APIs and dartpy bindings expose
   DART-owned domains, method families, policies, diagnostics, and value types,
   not upstream project names, solver registries, ECS storage, backend resources,
   or reverse-pass caches.
6. **Configuration surface.** Define the default `World`/options path, the
   advanced nested options, validation rules, serialization expectations, and
   diagnostics. The common path must be simple, and invalid or incompatible
   option combinations must fail before they produce misleading simulation
   results.
7. **Failure and fallback semantics.** Record unsupported-feature errors,
   fallback behavior, non-convergence handling, determinism requirements, and
   serialization/restart expectations before promoting a runtime path.

Reviewers should reject new solver work that cannot point to this intake
evidence. Cleanup of existing implementation is useful only when it also leaves
this gate clearer for the next paper implementation.

## Where Differentiable Solver Families Fit

Differentiability is a solver capability of the experimental `World`, not a
separate user-facing engine. The multi-solver architecture therefore has two
levels:

- **Domain selection** answers which physical model owns an entity. Rigid and
  articulated rigid entities still belong to the rigid-body domain; deformables
  still belong to the deformable domain; cross-domain interaction is still a
  coupler concern.
- **Method selection** answers which solver family advances that domain. The
  current differentiable work adds a solver-internal reverse pass to the
  generalized-coordinate boxed-LCP rigid-body path. A Dojo-style method would be
  another rigid-body solver family: maximal-coordinate or constrained-coordinate
  state, variational integration, hard-contact NCP/SOC friction, a primal-dual
  interior-point forward solve, and an implicit-gradient reverse pass.

That placement gives DART a clean relation between the existing and proposed
paths:

| Path                                      | Domain              | Forward method                                  | Gradient method                                | Relationship                                                                  |
| ----------------------------------------- | ------------------- | ----------------------------------------------- | ---------------------------------------------- | ----------------------------------------------------------------------------- |
| Existing rigid-body default               | Rigid / articulated | Semi-implicit dynamics + existing contact path  | none                                           | easy path and compatibility baseline                                          |
| PLAN-110 boxed-LCP/Nimble-style path      | Rigid / articulated | Generalized-coordinate boxed LCP contact solve  | active-set LCP implicit differentiation        | first opt-in differentiable rigid solver; default remains off                 |
| Planned Dojo-style evaluation             | Rigid / articulated | Variational maximal/constrained coordinate NCP  | IPM/KKT implicit differentiation               | possible second opt-in rigid solver family after an internal spike            |
| PLAN-081/PLAN-104 deformable solver paths | Deformable          | IPC/VBD-family deformable methods               | deferred per-solver/coupler differentiation    | separate domain; later coupled to rigid solvers through pairwise couplers     |
| PLAN-082 variational-integrator work      | Rigid / articulated | Variational integration for articulated systems | not sufficient by itself for contact gradients | shared integration rationale; Dojo adds contact/NCP/IPM and differentiability |

The public API should not expose a `DojoWorld`, `DojoSolver`, solver registry, or
Dojo.jl dependency. If promoted, users would opt in through DART-owned
domain-scoped options that request capabilities such as variational integration,
hard-contact NCP, interior-point contact solve, and analytic differentiability.
The `World` maps those policies to an internal solver or reports an
unsupported-capability error. Internally, the Dojo-style solver would plug into
the same lifecycle as every other solver: finalize model data, allocate
solver-owned state/cache, emit prepare/pre-couple/post-couple compute nodes, and
publish only DART-owned state/control/contact/derivative value objects through
the facade.

## Compute-Graph Integration

Solvers and couplers express their work as compute-graph nodes with explicit
dependencies and stage metadata; the executor (sequential reference, parallel,
or a future backend) runs the graph. This keeps the dynamics backend-neutral and
reuses the existing scheduling, profiling, and DOT-visualization surfaces. The
substep schedule is encoded as graph dependencies (pre-couple nodes precede
couple nodes precede post-couple nodes), not as hard-coded call order, so the
executor remains free to parallelize independent solver work within a phase.

The compute graph stays an implementation/extension surface. Backend types, task
systems, devices, streams, and memory pools are never part of the public solver
contract (see the compute-surface rules in the public-facade doc).

## Public Facade Rules

- Do not expose `Solver`, `Coupler`, `PhysicsDomain`, or schedule phase types as
  required public types in DART 7. Select behavior by documented method-family
  names and policy value objects.
- Keep solver options centered on the `World` and public body/domain handles:
  simple defaults and presets for common users, method-specific nested option
  objects for advanced users, and validation for incompatible combinations.
- Expose dynamics through entity handles and state/control/contact views with
  documented ownership and freshness, never through the registry.
- Default selection must keep the easy path (`World` + `addRigidBody` /
  `addMultiBody` + `step`) free of solver vocabulary.
- Custom solver/coupler plugins that cross shared-library or Python-callback
  boundaries are deferred until a dedicated ABI/lifetime/threading design
  exists.

## What This Architecture Defers

- the concrete contact/constraint solver internals (LCP formulation, friction
  model) — owned by the rigid-body solver implementation and its tests;
- deformable, particle, and fluid solvers and their couplers;
- model/state batching (replicated worlds, `n_envs`) and differentiable
  rollouts;
- a public solver/coupler registration API for third-party methods;
- accelerator backends and async/batched execution.

Each becomes real only with its own tests, docs, examples, and API-boundary
evidence, and only the backend-neutral, facade-safe subset is eligible for DART
8 promotion.

## Design Rationale

- Keeping dynamics in solvers and composition in the `World` lets DART add
  methods and domains without reshaping the user object model, which is the
  whole point of the experimental staging namespace.
- Domain-driven assignment gives a uniform `add*` API across radically different
  physical models while keeping geometry and physics orthogonal.
- Pairwise, swappable couplers avoid a monolithic coupling object that reaches
  into every solver's internals; that monolith is the main maintainability trap
  observed in existing multi-physics engines, and it scales poorly as domains
  multiply.
- Substep windowing is the minimal shared primitive that lets heterogeneous
  solvers interact without each knowing about the others, and it maps cleanly
  onto the compute-graph dependency model DART already has.
- Model/State/Control/Contacts separation is the precondition for batching and
  differentiability; designing for it now avoids a disruptive rewrite later,
  while the easy path keeps it hidden.
- Expressing solver work as compute-graph nodes keeps the execution backend
  replaceable and reuses existing profiling and visualization.

## Verification Expectations

Docs-only edits use the docs-only gate set from `docs/ai/verification.md`.

Implementation PRs that realize parts of this architecture should include:

- `pixi run lint`;
- `pixi run build`;
- focused C++ tests under `tests/unit/simulation/experimental/`;
- `pixi run check-api-boundaries` when public headers or dartpy bindings change;
- `pixi run test-py` when Python bindings are affected;
- benchmark evidence when a change claims a performance or scalability gain;
- changelog and migration notes when a legacy DART 6 surface is affected.

Reviewers should reject user-facing APIs that leak solver types, coupler types,
ECS storage, component types, execution-backend names, or registry access.
New solver/paper PRs should also link the solver-family intake evidence above
from the owner plan or dev-task resume surface.
