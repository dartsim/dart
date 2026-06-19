# Simulation C++ API

## Status

Accepted DART 7 API shape. This document owns durable API-shape rationale for
the C++ `dart::simulation` surface under `dart/simulation/**`.

DART 7 treats this API as the official clean-break simulation surface. The
legacy DART 6 simulation API is removed from `main` rather than carried beside
the new API; compatibility and parity evidence for that API live on
`release-6.*` branches.

The companion Python binding design lives in
[`simulation_python_api.md`](simulation_python_api.md).

## Purpose

The C++ DART 7 simulation API gives DART a clean public simulation surface for
the DART 7 clean break. The API should expose research-facing physics concepts
and stable extension points without exposing ECS storage,
implementation components, or backend execution details.

The core design sentence is:

> The C++ simulation API is the DART 7 clean-break facade, not a public contract for the underlying ECS
> implementation.

## Design Principles

### Progressive Disclosure

The common C++ path should fit in a short example:

```cpp
namespace sx = dart::simulation;

sx::World world;
auto box = world.addRigidBody("box", sx::RigidBodyOptions{});

world.enterSimulationMode();
world.step(100);
```

Advanced users can still opt into explicit validation, state spaces, custom
pipelines, executor injection, and diagnostics. Common users should not need to
understand storage, scheduling, component categories, or backend plumbing
before they can run a simulation.

### DART Vocabulary

Use DART and robotics vocabulary: worlds, bodies, multibodies, links, joints,
frames, DOFs, actuators, state, control, and compute stages. Public names
should describe physics concepts and algorithm families rather than
implementation containers.

Solver, backend, preset, and example names should use algorithm, approach,
paper, or DART-owned domain names instead of other engine or project names.
Prefer names that describe the method, such as articulated-body,
semi-implicit integration, projected Gauss-Seidel, XPBD, implicit time
stepping, sequential execution, or parallel execution. A compatibility bridge
may document what it imports or exports, but the primary C++ API name should
describe the method rather than the originating engine.

### Public Facade Before Implementation Coverage

The API should expose stable simulation concepts, not every type used by the
implementation. If the only available implementation type is a component,
registry handle, scheduler object, backend resource, or `detail` type, the
public API waits for a wrapper, value object, view, or handle with documented
ownership and lifetime.

### Stable Facade, Replaceable Internals

DART 7 clean-break promotion establishes a stable facade: public headers,
exported symbols, Doxygen behavior, examples, dartpy bindings, and migration
notes become the compatibility contract.

That stable facade should leave DART free to change implementation details:

- add algorithms, solvers, or multi-physics stages behind existing public
  concepts;
- improve scalar CPU, SIMD, multi-core, or accelerator implementations;
- choose internal compute paths such as CUDA, Metal, Vulkan compute, ROCm/HIP,
  LLVM/JIT code generation, or future backends from benchmark evidence;
- replace storage, scheduling, collision, sensor, rendering-prep, or solver
  internals without changing user object names;
- preserve source compatibility, or follow the deprecation, migration, and
  removal policy for the active major release line.

Backend and runtime names may appear in build options, diagnostics, profiles,
benchmark reports, or developer docs. They should not become required public
type names, solver names, namespace names, or object identities unless a later
design promotes a backend API intentionally.

### Scalar Precision Policy

The current `World` facade should remain double-precision unless a later
scalar-instantiation design proves otherwise. The existing scalar-generic
compute lessons remain useful for internal kernels, SIMD, autodiff experiments,
and future explicit instantiations, but they are not themselves a public
`World` precision contract.

Keep scalar-generic internals from becoming a one-way door where reasonable:
avoid hard-coding storage, handle, package, or boundary decisions that would
make a future scalar-instantiation plan unnecessarily invasive.

Do not promote scalar precision by forking the user-facing `World` identity or
by making backend/runtime precision names part of the required public type name.
A later public scalar-precision design must first define:

- concrete C++ instantiation ownership for `World`, options, handles, state,
  derivative, serialization, and package/export symbols;
- finite-difference, determinism, collision, Python, and installed-package
  gates for every advertised scalar;
- migration behavior for existing double-backed code; and
- API-boundary checks proving no solver, backend, ECS, or tensor-framework
  implementation type leaks into promoted headers.

Until those gates exist, scalar-generic rewrites should instantiate only the
supported public scalar and keep alternate precision or autodiff scalar choices
behind internal tests, benchmarks, or explicitly experimental developer
surfaces.

### One World, Multiple Runtime Intents

The same `World` should support full physics and kinematics-only workflows.
Motion-planning collision checks, externally driven robot playback, digital
twins, visualization, camera or marker sensors, and controllers should reuse
the same topology, frame, and object model instead of forcing users into a
separate kinematic scene API.

The public distinction should be pipeline intent: full dynamics stepping,
kinematics-only updates, collision query preparation, sensor sampling, or
rendering synchronization. Runtime shortcuts should not fork the user-facing
object model.

### Deterministic Sync, Optional Async

Synchronous stepping is the reference behavior. `World::step()` must remain
deterministic, easy to test, and complete when it returns, with fresh outputs
for the stages it executed.

Asynchronous stepping may be added for throughput, UI responsiveness, server
workloads, or batched simulation, but it must compose around the same world,
pipeline, state, and synchronization concepts. Async APIs should make
ownership, completion, cancellation, and result visibility explicit. They
should not become the only path to high-performance simulation or change the
semantics of synchronous stepping.

This matches the useful external split between deterministic stepping and
runtime scheduling policy. Drake's `Simulator::AdvanceTo`, MuJoCo's `mj_step`
and `mj_forward`, PyBullet's `stepSimulation` versus real-time mode, Gazebo
server run control, and Isaac's sync/async application modes all keep
advancement semantics distinct from event-loop or background execution. See
[Drake Simulator](https://drake.mit.edu/pydrake/pydrake.systems.analysis.html),
[MuJoCo functions](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html),
[PyBullet quickstart](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html),
[Gazebo Server](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1Server.html),
and
[Isaac Lab SimulationContext](https://isaac-sim.github.io/IsaacLab/v2.0.0/_modules/isaaclab/sim/simulation_context.html).

### Tree Topology Plus First-Class Closures

Articulated systems should keep a tree-shaped `Multibody` as the owner for
links, joints, names, and state indexing, then represent closed chains as
explicit first-class closure constraints between symmetric public endpoints.

This keeps the common serial-chain and branched-tree API simple while allowing
closed-chain mechanisms to participate in kinematic projection, residual
diagnostics, and dynamic constraint solving. A closure is not a second parent,
a fake child joint, or an exposed solver row. It is public topology plus
runtime policy, with the implementation free to choose the constraint
formulation behind that facade.

Closure endpoints should use symmetric names such as `frameA`/`frameB` or
`endpointA`/`endpointB`, never `parent`/`child`. Runtime policy such as enabled
state, residual-only reporting, kinematic projection, and dynamic solving
should be separate from topology so a closed-chain definition can stay stable
while solver participation changes by pipeline stage.

### Constraint Semantics Before Solver Mechanics

Closed-chain, contact, limit, and coupling APIs should name the physical or
kinematic relation first: rigid closure, point coincidence, distance closure,
axis alignment, joint coupler, gear constraint, contact, or limit. Solver
controls should use algorithmic terms such as position projection, velocity
projection, Baumgarte stabilization, compliance, damping, tolerance, maximum
iterations, range-space solve, null-space projection, or sequential impulse.

The public API should not name a constraint or solver after another engine. A
runtime may import a model format or choose an internal implementation inspired
by an engine, paper, or backend, but users should configure DART concepts and
algorithm families rather than implementation ancestry.

### Fresh Results, Explicit Work Placement

The long-term API should preserve the safety of DART 6 lazy evaluation without
making the DART 6 dirty-flag network the public or required implementation.
Users should be able to set state and then read transforms, query collisions,
or render without remembering an implementation-specific cache update order.
The overall rule is implicit freshness with explicit work placement.

The best long-term shape is a hybrid:

- ordinary object queries return fresh values by default;
- `World::step()` and kinematics-only tick/sync methods guarantee freshness for
  the stage bundle they execute;
- explicit synchronization methods remain available for controllers,
  planners, rendering loops, and batched reads that want predictable work
  placement;
- advanced unchecked reads, if added, are visibly named and documented as
  possibly stale.

This preserves intuitive reads while allowing the implementation to use dirty
flags, generation counters, dependency graphs, staged cache owners, or
backend-specific batch updates internally.

## Scope

This design covers the supported shape of the experimental C++ API as it moves
from opt-in status to the official DART 7 clean-break simulation API.

It includes:

- namespace and header transition rules;
- public object model and ownership semantics;
- design-mode and simulation-mode lifecycle;
- closed-chain kinematic and dynamic structure;
- naming rules for solvers, backends, presets, and examples;
- state and array ownership expectations;
- kinematics-only execution for collision queries, kinematic state updates,
  visualization, and visual or kinematic sensors;
- stable public API rules that allow internal algorithm, solver, multi-physics,
  and compute-backend changes;
- compute-graph exposure boundaries;
- DART 7 promotion and legacy API removal rules.

It does not track active implementation tasks, release priority, or migration
checklists. Those belong in `docs/plans/`, `docs/dev_tasks/`, release notes, or
the release roadmap.

## Non-Goals

- Do not make the DART 7 experimental namespace a long-term compatibility
  namespace after clean-break promotion.
- Do not keep the legacy DART 6 simulation API beside the promoted DART 7 API.
- Do not expose `entt::registry`, `entt::entity`, `comps`, component category
  types, raw component storage, `detail`, `internal`, backend task systems, GPU
  devices, streams, memory pools, or solver registries as user API.
- Do not expose backend-specific scheduler, accelerator, or rendering concepts
  until a later plan and benchmark gate justifies a stable public contract.
- Do not require users to include implementation headers to use public
  simulation concepts.
- Do not require full dynamics stepping for workflows that only need frame
  updates, collision queries, visualization, or visual/kinematic sensors.
- Do not name public APIs after implementation backends such as GPU, compiler,
  or rendering runtimes unless a separate backend API is intentionally
  promoted.
- Do not expose dirty flags, cache bits, registry versions, or backend cache
  ownership as user-facing API.

## Current Evidence

- `dart::simulation::World` owns the new world lifecycle,
  topology construction, stepping, compute-executor overloads, serialization,
  and registry-backed storage.
- `World::sync(WorldSyncStage::Kinematics)` executes the kinematics graph
  without the default rigid-body integration stage, and its executor overload
  lets the same kinematics-only path use backend-neutral compute execution.
  `World::updateKinematics()` remains available in DART 7 as the existing
  synchronization spelling. The promotion target is fresh-by-default ordinary
  queries plus named synchronization hooks for predictable batching.
- Default `World::step()` uses a content-aware built-in schedule, selected from
  `WorldOptions` / setter policy: free-rigid sequential impulse or IPC, the
  semi-implicit or variational multibody family, the unified constraint/contact
  path when articulated structures are present, deformable dynamics only when
  deformable bodies exist, and a final kinematics refresh. The schedule is an
  internal `WorldStepPipeline` of cached stages, so common users select
  capability names and policies rather than stage objects. The batched
  rigid-body integration stage remains an internal explicit unconstrained SoA
  path for parity tests, SIMD/data-locality work, and future device prototype
  evidence. Executor/pipeline overloads allow caller-owned execution policy and
  custom user stages through the public `WorldStepStage` /
  `WorldStepPipeline` contract, while DART-owned concrete family stages stay
  out of the installed public headers.
- `RigidBodyOptions` already represents user-facing rigid-body initialization
  data: mass, inertia, pose, and velocity.
- `Frame`, `FreeFrame`, `FixedFrame`, `Multibody`, `Link`, and `Joint` provide
  first-class handle concepts over the experimental storage.
- The implemented DART 7 `Multibody`, `Link`, and `Joint` API is currently
  tree-shaped, with public `JointSpec` construction, joint type, axis,
  parent/child link access, rigid-body endpoint access for public rigid-body
  fixed joints, DOF count, and generalized position/velocity access. `World`
  now owns `LoopClosure` handles with symmetric frame endpoints, semantic
  closure families, offsets, runtime participation policy, residual
  diagnostics, lookup, validation, and serialization. Closure kinematic
  projection and dynamic closure solving remain staged design targets. Active
  projection or solve policies are rejected at runtime until a compatible stage
  exists, so policy metadata cannot silently overpromise solver participation.
- DART 6-style downstream closed-chain examples use a tree skeleton plus
  solver constraints or mimic/coupler metadata. Examples such as
  `examples/rigid_loop`, `examples/coupler_constraint`, and
  `examples/mimic_pendulums` are reference material for import compatibility
  and semantics, not the DART 7 clean-break API shape.
- `World::getRegistry()` is a DART 7 implementation escape hatch for tests and
  internal bring-up. It is explicitly excluded from clean-break promotion unless
  a later design creates a stable storage-inspection API.
- `StateSpace` provides a bindable, storage-independent value object for named
  flat-vector metadata.
- The experimental compute benchmark includes both world kinematics updates and
  full world stepping, which is the right evidence shape for measuring
  kinematics-only performance gains against full physics.
- Native collision already has standalone world/query concepts with explicit
  update and query operations; the DART 7 simulation API still needs a
  public owner bridge before those become part of the new world contract.
- The legacy DART 6-era dynamics API uses lazy forward-kinematics updates:
  transform getters compute on demand after position/velocity/acceleration
  writes mark dirty caches. That gives safe fresh reads, but it also spreads
  dirty-flag bookkeeping across frames, joints, body nodes, Jacobians, skeleton
  caches, shape caches, and support caches.
- `docs/onboarding/api-boundaries.md` requires experimental APIs to have docs,
  tests, and ownership while keeping component storage, backend plumbing, and
  implementation namespaces out of public contracts.
- `docs/onboarding/release-roadmap.md` defines DART 7 as the clean break that
  removes the legacy DART 6 API and promotes the new simulation API after
  parity gates pass.

## Namespace And Header Plan

DART 7 keeps the new simulation API under:

```cpp
namespace dart::simulation
```

and headers under:

```text
dart/simulation/**
```

DART 7 promotes the supported subset into the official stable simulation API.
The exact final header layout can be decided during promotion, but the stable
surface should preserve the same user-facing concepts and remove the DART
6-era simulation API rather than wrapping it as a compatibility layer.

The promoted C++ API should satisfy these conditions:

- public headers are outside `detail/`, `internal`, and component-storage
  folders;
- exported symbols use the appropriate component API macro when they cross a
  shared-library boundary;
- public signatures use public value types, handles, views, or spans instead of
  implementation containers;
- examples use only the promoted public headers and namespace;
- Doxygen describes behavior, ownership, lifetime, and error handling.

The expected public header families are:

| Header family                | Purpose                                                     | Promotion gate                                                  |
| ---------------------------- | ----------------------------------------------------------- | --------------------------------------------------------------- |
| `world`                      | World lifecycle, stepping, construction, and serialization. | No public signature depends on component storage.               |
| `objects`                    | Rigid bodies, multibodies, links, joints, and frames.       | Handles document ownership, validity, and cross-world behavior. |
| `state`                      | State spaces, state/control values, and state views.        | Explicit copy/view/write-back contracts and tests.              |
| `compute`                    | Stages, pipelines, executors, profiles, and graph metadata. | Backend-neutral interfaces and benchmark evidence.              |
| `contacts`                   | Collision results and constraint/contact views.             | Public buffers/views with allocation and attribute rules.       |
| Future `sensors`/`rendering` | Sensor owners, snapshots, viewers, and renderers.           | Separate ownership, sync, lifetime, and threading contracts.    |

Header families are conceptual. The exact file layout can change during DART 7
promotion, but public examples should never require implementation folders.

## Public Object Model

| Concept                | DART 7 experimental owner                                                                                                                                   | DART 7 promotion target                                                                                                               |
| ---------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `World`                | Owns topology, time, frame count, stepping, serialization, and compute entry points.                                                                        | Official simulation world.                                                                                                            |
| `RigidBody`            | World-owned handle for a single rigid object and frame.                                                                                                     | Public rigid body handle with transform, velocity, inertial, force, and torque APIs, plus geometry/material APIs once wrappers exist. |
| `RigidBodyOptions`     | Public value object for mass, inertia, pose, and velocity initialization.                                                                                   | Stable construction/configuration value object.                                                                                       |
| `Multibody`            | World-owned handle for articulated rigid-body topology.                                                                                                     | Official articulated-body concept, with final naming chosen during promotion.                                                         |
| `Link`                 | Body in a multibody kinematic tree and frame participant.                                                                                                   | Public link handle.                                                                                                                   |
| `Joint`                | Connection between links or public rigid-body fixed-joint endpoints with type, axes, endpoint access, DOF count, and generalized position/velocity vectors. | Public joint handle with complete state/control, limits, transforms, and diagnostics once wrappers exist.                             |
| `LoopClosure`          | Explicit spatial closure between two public frames, links, or bodies.                                                                                       | Public closed-chain handle with symmetric endpoints, diagnostics, and runtime solve policy separated.                                 |
| `Frame`                | Spatial reference frame with transform queries.                                                                                                             | Stable frame concept for bodies, links, and user frames.                                                                              |
| `StateSpace`           | Named flat-vector metadata independent of storage.                                                                                                          | Stable state metadata surface for optimization and control workflows.                                                                 |
| Compute graph concepts | Experimental graph, executor, metadata, profile, and pipeline hooks.                                                                                        | Stable extension points only for backend-neutral concepts that pass benchmark and API-boundary gates.                                 |

The public API should use explicit C++ concepts even when the implementation
stores data in ECS components. Component names are implementation details.

Public handles should document:

- owner world identity and whether cross-world use is rejected;
- validity after `World::clear()`, object removal, rebuild, and world
  destruction;
- whether equality compares identity, value, or handle target;
- whether accessors throw, return status, or return empty values after
  invalidation;
- thread-safety for read-only queries and mutation.

Every first-class handle should expose a public validity query before users
need to inspect implementation storage to understand lifecycle effects.

## Lifecycle

The C++ API keeps topology mutation separate from simulation execution:

```cpp
namespace sx = dart::simulation;

sx::World world;
auto body = world.addRigidBody("box", sx::RigidBodyOptions{});
auto robot = world.addMultibody("arm");

world.enterSimulationMode();
world.sync(sx::WorldSyncStage::Kinematics);
world.step();
```

DART 7 may keep existing camelCase methods while the experimental API matures.
For clean-break promotion, public names should follow the DART 7 public API
naming policy for the target namespace and headers. Compatibility wrappers for
the legacy DART 6 simulation API are not part of the stable DART 7 surface.

Design-mode errors, invalid handles, topology mismatches, and cross-world
object use should fail through documented exceptions or status-returning
functions rather than assertions in user paths.

### Closed-Chain Structure

Closed-chain mechanisms should be modeled as explicit loop-closure structure,
not as duplicated bodies, fake tree edges, or exposed internal constraint
components. The initial tree of `Multibody`, `Link`, and `Joint` remains useful
for ownership, naming, state indexing, and articulated-body algorithms; closure
edges add graph structure on top of that tree.

DART 7 now stages the topology, runtime-intent, and residual-diagnostic part
of this model:
`World::addLoopClosure(...)` returns a first-class `LoopClosure` handle with
symmetric endpoint frames, semantic family, endpoint offsets, name lookup,
count queries, validation, serialization, and a backend-neutral
`LoopClosureRuntimePolicy`. `LoopClosure::computeResidual()` returns explicit
closed-chain residual diagnostics without exposing solver rows. Constrained
kinematic projection and dynamic solving remain clean-break target concepts to
stage behind the experimental namespace before promotion.

The staged public C++ shape is a DART-owned handle and spec:

```cpp
auto closure = world.addLoopClosure(
    "four_bar_closure",
    sx::LoopClosureSpec{
        .frameA = groundFrame,
        .frameB = couplerLink,
        .family = sx::LoopClosureFamily::Rigid,
        .offsetA = Eigen::Isometry3d::Identity(),
        .offsetB = Eigen::Isometry3d::Identity(),
    });
```

Runtime policy is public metadata while projection and solving remain staged:

```cpp
closure.setRuntimePolicy(sx::LoopClosureRuntimePolicy{
    .enabled = true,
    .kinematics = sx::ClosureKinematicsPolicy::Project,
    .dynamics = sx::ClosureDynamicsPolicy::Solve,
});
```

Residual diagnostics are explicit queries rather than implicit side effects:

```cpp
auto residual = closure.computeResidual();
```

The minimal world-owned construction surface should be:

```cpp
sx::LoopClosure World::addLoopClosure(
    std::string_view name, const sx::LoopClosureSpec& spec);
std::optional<sx::LoopClosure> World::getLoopClosure(
    std::string_view name);
bool World::hasLoopClosure(std::string_view name) const;
std::size_t World::getLoopClosureCount() const;
```

The exact type names can change before promotion, but the public contract
should distinguish:

- symmetric closure endpoints from tree parent/child relationships;
- closure topology, which names the endpoint frames and residual family;
- runtime participation, such as enabled/disabled state, kinematic projection,
  residual-only reporting, and full dynamic constraint solving;
- topology validation and finalization from per-step runtime activation;
- spatial closure families such as rigid, distance, point, and axis closures
  without naming them after an implementation engine;
- scalar multibody coupling families, such as joint couplers and gear
  constraints, as related but separate public concepts rather than overloading
  every constraint as a `LoopClosure`;
- activation, enable/disable, tolerance, stabilization, and diagnostic
  metadata as public value fields rather than backend component fields.

Closed-chain APIs must define ownership and lifetime. World-owned closures are
the conservative default because endpoints may span multibodies, rigid bodies,
and the world frame. A `Multibody` convenience method may forward to the world
only when both endpoints are in the same owner. Cross-world endpoints must be
rejected, and removal, serialization, topology rebuilds, and `World::clear()`
must document handle invalidation.

Topology validation must reject degenerate endpoint pairs, cross-world
endpoints, unsupported closure families for the selected pipeline, and
inconsistent initial guesses when the policy requires an initially satisfied
closure. After validation or simulation-mode finalization, adding or removing
closures should require an explicit reset, rebuild, or clear operation. First
`World::step()` may perform this freeze in the common path only if the timing
and failure modes are documented.

Closed-chain APIs must define how closures affect DOF counting, state-space
metadata, loop validation, serialization, collision filtering, and diagnostics.
The expected baseline is that `Multibody::getDOFCount()` reports the
underlying tree generalized-coordinate dimension; closures add residual rows,
constraint metadata, active flags, tolerances, convergence status, and
force/impulse diagnostics. If a future API exposes independent constrained
coordinates, that should be a separate state-space view rather than silently
changing tree DOF counts.

A kinematics-only pipeline should state whether it projects closure errors,
reports residuals only, or requires the caller to select an explicit projection
stage. Dynamic closure behavior belongs to a named constraint or
implicit-dynamics stage, not to ordinary frame-cache refresh.

Diagnostics should be queryable without exposing solver rows directly. The
current runtime policy records enabled state plus residual-only, projection, or
solve intent, while `LoopClosureResidual` reports the residual vector and norm,
world-frame coordinate convention, active/enabled state, and whether solved
force or impulse estimates are available. A future diagnostic value can add
projection or solve convergence status, iteration count, tolerance used, and
force or impulse estimates expressed in a documented endpoint frame. If the
active pipeline only reports residuals, force and impulse fields should be
absent or explicitly marked unavailable.

The API should keep the useful lessons from existing engines without inheriting
their public vocabulary:

| Engine/API pattern     | Lesson for DART's public API                                                                                         |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------- |
| MuJoCo equality models | Keep loop closures outside the kinematic tree as named residuals that can feed dynamics.                             |
| Drake multibody plant  | Expose named constraints and IDs, and make the same model constraints usable by kinematics-only optimization paths.  |
| PhysX articulations    | Preserve a reduced-coordinate tree and close loops with constraints, but document approximation and topology limits. |
| Isaac Sim rigging      | Importers may need to cut a closed CAD loop and add mimic/coupling structure, but that should be importer policy.    |
| Gazebo/SDFormat graphs | Model formats can express graph loops; runtime APIs still need explicit validation, cut-joint, and residual policy.  |
| PyBullet facade        | A stable client API can hide physics/render engine replacement, but DART should expose typed closure concepts.       |
| CoppeliaSim routines   | Kinematics-only loop solving should be first-class, separate from dynamic stepping.                                  |

- MuJoCo models loop joints through equality constraints with residuals
  `r(q) = 0`, including connect and weld constraints outside the kinematic
  tree. The same documentation warns that using equality constraints as normal
  tree joints is slower and less accurate. That supports the DART distinction
  between tree joints and explicit closure residuals while keeping closure
  forces available to dynamics. See
  [MuJoCo computation: equality constraints](https://mujoco.readthedocs.io/en/stable/computation/index.html#equality)
  and the
  [MuJoCo XML equality reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality).
- Drake exposes named multibody constraints such as ball, weld, distance, and
  coupler constraints with stable IDs. Its inverse-kinematics API can add
  supported plant constraints into a mathematical program, which supports
  DART's goal that the same closure topology can serve kinematics-only
  projection and full dynamics. See
  [Drake MultibodyPlant](https://drake.mit.edu/pydrake/pydrake.multibody.plant.html)
  and
  [Drake inverse kinematics](https://drake.mit.edu/pydrake/pydrake.multibody.inverse_kinematics.html).
- PhysX articulations are reduced-coordinate trees; loops are closed by adding
  rigid-body joints between articulation links. PhysX also documents that
  articulation topology changes require rebuilding scene data and that stiff
  coupled constraints can fail to satisfy every constraint. That supports
  explicit DART topology finalization and honest approximation diagnostics. See
  [PhysX articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/Articulations.html).
- Isaac Sim's closed-loop rigging guidance teaches users to break a closed
  articulation chain and add mimic joints or other coupling. That supports DART
  importers choosing cut joints while keeping the public API expressed as
  closures and couplers. See
  [Isaac Sim closed-loop rigging](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/rig_closed_loop_structures.html).
- Gazebo's four-bar tutorial states that URDF's tree structure cannot express
  closed loops while SDFormat can because it is graph structured. SDFormat's
  joint schema also exposes `must_be_loop_joint` to force a cut in the
  multibody graph. That supports explicit DART graph validation and cut-policy
  import diagnostics. See
  [Gazebo kinematic loop tutorial](https://classic.gazebosim.org/tutorials?cat=&tut=kinematic_loop)
  and
  [SDFormat joint physics](https://sdformat.org/spec/1.11/joint/).
- PyBullet's quickstart describes a Python facade over a C API intended to be
  independent of the underlying physics and render engines. That supports
  DART's stable facade principle, while DART should prefer typed closure
  handles over generic integer constraint IDs. See
  [PyBullet quickstart](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html).
- CoppeliaSim's standalone kinematics routines support Jacobian-based
  forward/inverse kinematics for mechanisms containing nested loops. That
  supports a DART kinematics-only closure projection API independent of full
  dynamic stepping. See
  [Coppelia kinematics routines](https://manual.coppeliarobotics.com/en/coppeliaKinematicsRoutines.htm).

### Topology And Runtime Separation

Topology construction is design-mode work. Simulation mode owns runtime state,
contact buffers, caches, and compute resources. Topology changes after
simulation starts require explicit reset, rebuild, or clear semantics, and the
API must document which handles and views remain valid.

The common path should let users build a world and call `World::step()`.
Explicit validation/finalization calls remain useful for deterministic
allocation, diagnostics, and controlled failure timing:

```cpp
world.validateTopology();
world.enterSimulationMode();
world.reset();
world.step(100);
```

For DART 7, `World::step()` and `World::step(count)` enter simulation mode as a
common-path convenience, while `World::sync(WorldSyncStage::Kinematics)` is the
preferred explicit simulation-mode synchronization hook for predictable
kinematics-only work placement. `World::updateKinematics()` remains a DART 7
spelling over the same stage. Clean-break promotion must keep the C++ and
Python lifecycle rules identical: zero-count repeated stepping is a no-op,
positive-count stepping validates/finalizes once before the first step, and
topology mutation after finalization requires documented reset, rebuild, or
clear behavior. Repeated stepping should reuse the same executor and pipeline
state when an overload exposes them.

Rendering or application event loops should not be prerequisites for physics
stepping. Future viewer and renderer objects must own their own synchronization
and lifetime contracts.

### Async Stepping Shape

The synchronous `World::step()` path remains the semantic reference. A future
async API should be an explicit scheduling layer around the same stage and
state model, for example a DART-owned step handle or future-like object that
documents completion, cancellation, error propagation, and when outputs become
visible.

Async execution should be backend-neutral. Public names should describe
runtime policy such as synchronous, asynchronous, real-time, fixed-rate, or
batched execution rather than naming CUDA, Metal, Vulkan, ROCm, LLVM, or
external engines. Backend details belong in capability queries, diagnostics,
profiles, and intentionally designed backend APIs.

### Kinematics-Only Runtime

The same `World` should support runtime workflows that do not integrate
dynamics. This keeps motion-planning collision checks, externally driven robot
playback, digital twins, visualization, camera or marker sensors, and
controllers on the same topology, frame, and object model as full physics.

The public distinction should be pipeline intent, not a separate world type:

```cpp
world.enterSimulationMode();
joint.setPosition(q);
world.sync(sx::WorldSyncStage::Kinematics);
```

When callers need predictable work placement or alternate execution policy,
the same kinematics-only path accepts the backend-neutral executor facade:

```cpp
compute::ParallelExecutor executor;
world.sync(sx::WorldSyncStage::Kinematics, executor);
```

`World::updateKinematics()` delegates to this sync stage in DART 7. A future
convenience wrapper may name a kinematics-only tick or pipeline, but the durable
C++ contract is:

- kinematics-only execution updates frames and caches without integrating
  forces, velocities, or rigid-body state;
- collision query stages may update broad-phase and shape-transform data, but
  do not solve contact impulses or constraints;
- visualization and visual/kinematic sensors consume the same frame caches as
  full physics;
- time and frame counters advance only through documented tick semantics;
- common queries stay fresh by default, while advanced bulk loops can call an
  explicit synchronization hook once before many reads;
- benchmark evidence compares the kinematics-only path against the full
  physics path for the same scene before advertising performance gains.

### Freshness And Cache Semantics

The long-term API should preserve the safety of DART 6 lazy evaluation without
making the DART 6 dirty-flag network the public or required implementation.
Users should not need to know whether freshness is implemented with dirty
flags, generation counters, dependency graphs, staged cache owners, or
backend-specific batch updates.

The C++ public contract should be:

- ordinary object queries return fresh values by default;
- joint position writes invalidate affected kinematic outputs internally; users
  do not observe dirty flags, cache bits, or registry versions;
- the current DART 7 joint position/velocity wrappers drive open-chain
  forward-kinematics refreshes for standard tree joints, but closed-chain
  projection remains a staged solver capability;
- `World::step()` and kinematics-only tick/sync methods guarantee freshness for
  the stage bundle they execute;
- explicit synchronization methods remain available for controllers,
  planners, rendering loops, and batched reads that want predictable work
  placement;
- advanced unchecked reads, if added, are visibly named and documented as
  possibly stale;
- expensive query APIs define whether they synchronize automatically, update
  only when stale, or require the caller to pass an update policy.

This hybrid keeps the easy path intuitive while allowing the implementation to
avoid recursive per-object dirty propagation in scalable workloads. Internally,
epoch or generation counters can make no-op freshness checks cheap, and compute
pipelines can refresh only the stage outputs requested by the workload.

Multibody-local link and joint enumeration is part of the public facade because
the `Multibody` owner already stores these handles in construction order. C++
snapshot APIs such as `getLinks()`, `getJoints()`, `getLinkNames()`, and
`getJointNames()` should return lightweight public handles or value data, not
raw entity IDs, component references, or backend storage views. Rich named
collection objects and bulk DOF views should wait until uniqueness, invalidation,
and state-owner contracts are documented.

Names used for owner-local lookup should be unique within their public owner:
`World` owns multibody, rigid-body, and loop-closure names, while `Multibody`
owns link and joint names. Autogenerated names should skip existing names so
explicit names cannot collide with later generated names. Cross-owner topology
mutation, such as adding a child link to one `Multibody` using a parent link
from another `Multibody`, should be rejected at the API boundary; closed-chain
relationships belong in explicit loop-closure objects instead.

World-owned named objects should expose a consistent lookup shape while richer
collection views mature: optional `get*()` lookup for a first-class handle,
`has*()` presence queries for branch-free validation, and count properties for
simple inspection. Construction-ordered world-level collection snapshots should
wait until `World` owns an explicit public ordering contract rather than using
registry iteration order.

## Public Facade Rules

The promoted API should expose public wrappers before exposing implementation
fields:

- `JointType` should become a public simulation type before Python or C++ user
  examples rely on it; users should not include component headers for joint
  enum values.
- Link and joint construction should use public option/spec value objects, not
  raw component structs; `JointSpec` now covers the parent-joint construction
  data shared by C++ and dartpy.
- Public rigid-body fixed joints should expose rigid-body endpoints through
  explicit handle accessors; `getParentLink()` and `getChildLink()` remain
  multibody-link accessors.
- Rigid-body pose, velocity, mass, inertia, force, torque, collision shape, and
  material access should be added through public methods before examples use
  those concepts; transform, velocity, mass, inertia, force, and torque now
  have public wrappers.
- Joint state, limits, effort/control commands, and transforms should be added
  through public methods before examples use those concepts; joint DOF count,
  generalized position/velocity, and open-chain FK refreshes now have public
  wrappers.
- World state access should use public state views or explicit copy/write-back
  APIs, not direct registry mappers.
- Public handles must document validity after `World::clear()`, entity removal,
  and world destruction.

### Construction Value Objects

Public construction should use DART-owned value objects rather than exposing
component structs. Examples include:

- `RigidBodyOptions` for mass, inertia, pose, and velocity;
- `JointSpec` for joint type and axis; future extensions should add joint
  limits and other joint-specific construction data here rather than exposing
  component storage;
- future material/contact/geometry/appearance value objects that keep source
  geometry, physical/contact behavior, inertial data, and visualization data as
  separate concepts;
- `WorldOptions` plus future `StepOptions` or executor options for local
  configuration.

Value objects should validate field names and units in Doxygen and tests. The
public API may offer convenience overloads, but examples should keep the value
object form visible for stable workflows.

## State, Views, And Ownership

The C++ API should avoid exposing mutable implementation containers. Preferred
public state shapes are:

- small value types for options and metadata;
- `Eigen` values or documented views for transforms and vectors;
- `std::span` or DART-owned view types for contiguous user-facing state;
- explicit copy/write-back transactions for mutable state snapshots;
- `StateSpace` metadata plus a future public world-state view for optimization
  and control workflows.

Any mutable view must document owner lifetime, invalidation, thread-safety, and
whether writes immediately affect the world or require an explicit commit.

Advanced APIs should distinguish topology, mutable state, mutable control, and
contact/collision data. `World::step()` can hide that separation in the common
path, but explicit control, rollout, and optimization APIs need stable owner
semantics. The primary buckets are:

- topology/model data: dimensions, names, geometry, joints, and parameters;
- state data: positions, velocities, time, caches, and solver work values;
- control data: targets, efforts, impulses, and user inputs;
- contact data: typed buffers/views produced by collision generation and
  consumed by solvers.

`World::getStateVector()` / `setStateVector()` and
`World::getControlVector()` / `setControlVector()` are the general dense world
view: dynamic rigid-body translations first, then multibody joint generalized
coordinates in baked dense order. The current differentiable rigid-body
translation reduction remains available through the scoped
`getRigidBodyStateVector()` / `setRigidBodyStateVector()` and
`getRigidBodyControlVector()` / `setRigidBodyControlVector()` accessors, so
callers can name that narrower slice explicitly.

Replay and scrubber workflows follow the same split. Opt-in live-world replay
should store only the mutable runtime state needed to restore an
already-simulated frame: time/frame metadata, active solver mode, positions,
velocities, forces, controls, diagnostics, and solver work values that affect
continuation from that frame. It should not duplicate topology, geometry,
material, asset, or static construction data per frame. If that static layout
changes after recording, restore should fail with a clear layout-incompatibility
error instead of attempting best-effort partial replay.

Contact buffers should declare optional attributes before allocation when those
attributes change memory layout or cost. Public contact views should document
ordering, lifetime, matching behavior, and whether they remain valid after
`World::step()`.

### Rollout And Batch Shape

A future rollout API should be separate from live-world stepping. It may accept
explicit topology/model data, initial state, control sequence, output buffers,
and step count, but it should not expose devices, streams, memory pools, or
task-graph implementation types.

Batch shape conventions should be documented at the C++ layer before Python
adopts them. The first target should be homogeneous replicated worlds with a
leading world dimension, explicit per-world options, and explicit selection or
mask arguments. Heterogeneous topology batches are a later capability.

## Compute Surface

The compute graph is a valid experimental extension point, but clean-break
promotion should include only backend-neutral concepts:

- graph nodes and explicit dependencies;
- named stage bundles for full physics, kinematics-only updates, collision
  queries, sensor updates, and rendering prep;
- sequential execution as the reference path;
- executor injection through public abstract interfaces;
- stage metadata, domain/acceleration metadata, execution profiles, and DOT
  visualization;
- custom world-step stages and pipelines.

The installed compute stage surface is the extension contract, not the built-in
schedule catalog. `WorldStepStage`, `WorldStepPipeline`, executor interfaces,
metadata, and profile value types are public; concrete DART-owned family stages
and their large internal stats structs remain implementation headers. Facade
diagnostics should expose curated snapshots through `World` and profile value
types instead of requiring users to instantiate a built-in stage object.

The public API should not expose backend implementation types, raw task graph
types, GPU devices, streams, kernels, memory pools, transfer queues, or solver
registries until workload, benchmark, packaging, and API-boundary evidence make
them stable contracts.

Resource access metadata should remain diagnostic until a later scheduler
contract is intentionally designed and verified.

## Solver And Execution Policy

Solver and execution APIs should be documented by method capability, not by
backend or external project names. Public names should describe algorithms,
approaches, papers, or DART-owned policies.

The solver documentation should use a capability matrix:

| Capability         | Examples of documented values                                 |
| ------------------ | ------------------------------------------------------------- |
| Integration family | semi-implicit Euler, implicit Euler, variational integrator.  |
| Dynamics approach  | articulated-body method, constrained dynamics, XPBD.          |
| Constraint solve   | projected Gauss-Seidel, direct solve, barrier method.         |
| Coordinate support | maximal coordinates, generalized coordinates, mixed systems.  |
| Supported features | contacts, joints, actuators, soft constraints, closed chains. |
| Execution shape    | single world, kinematics-only, homogeneous batch, rollout.    |
| Differentiability  | unsupported, finite-difference checked, analytic, autodiff.   |

Do not promote solver registries, plugin loaders, backend-specific task
systems, or accelerator resource handles without a separate API design that
defines ownership, ABI, threading, diagnostics, and benchmark gates.

New solvers and multi-physics stages should be additive under DART-owned
capability names. Users should request method families or policies and receive
documented fallback behavior or unsupported-capability errors when the current
build lacks the required implementation backend.

For example, a future Dojo-style differentiable rigid-body path would not expose
`Dojo` as a public solver identity. It would be represented as a rigid-domain
method family with documented capabilities: maximal- or constrained-coordinate
state, variational integration, hard-contact NCP/SOC friction, an
interior-point contact solve, and analytic differentiability. That keeps it
under the same `World`/`MultibodyOptions` style as the existing
generalized-coordinate boxed-LCP/Nimble-style path and preserves method-based
fallback and unsupported-capability errors.

Capability selection is exposed through **domain-scoped value objects set as a
whole**, not a setter/getter per capability, so new capability fields are added
without growing the `World` method surface. The realized C++ shape is a
world-level construction grouping for defaults and policies
(`WorldOptions::rigidBodySolver`, `WorldOptions::multibodyOptions`,
`WorldOptions::contactSolverMethod`, `WorldOptions::contactGradientMode`, and
`WorldOptions::differentiable`) plus the existing interactive setters for the
policies that are safe to switch after construction (`World::setRigidBodySolver`,
`World::setMultibodyOptions` / `getMultibodyOptions`, and
`World::setContactGradientMode`). `MultibodyOptions {
MultibodyIntegrationFamily integrationFamily; }` maps onto the "Integration
family" matrix row. Selection is a typed enum mapped to an internal
representation on construction or set, so the per-step path carries no
configuration cost when a non-default family is not in use.

## Future Capability Shapes

These sections describe C++ target shapes, not guaranteed DART 7 APIs.

### Kinematic Queries And Collision

Kinematics-only workflows should support query stages that run after frame
updates and before optional visualization or sensor updates. The API should
define:

- frame-cache and shape-transform freshness before query results are produced;
- whether a collision or distance query updates broad-phase data structures;
- contact, distance, raycast, visibility, and marker-query result ownership;
- stale-query behavior when callers mutate poses without updating kinematics;
- performance counters that separate kinematics update cost, query update cost,
  and full physics step cost.

Collision queries in a kinematics-only pipeline are queries, not solvers. They
may produce contact or distance data, but they do not integrate impulses or
advance constraint state unless a full-physics stage is requested.

### Sensors

Sensors should be public owners or handles with typed configuration,
attachment, reset/update integration, and timestamped measurement snapshots.
The API should define:

- attachment targets such as frames, rigid bodies, links, joints, or contact
  selections;
- required state and contact attributes;
- update cadence and freshness semantics;
- units, coordinate frames, and timestamp source;
- validity after world reset, rebuild, object removal, and destruction.

Sensor docs should classify whether a sensor needs full physics, collision
query data, rendering data, or kinematics only. Visual and kinematic sensors
should be usable from a kinematics-only pipeline when their inputs are
available.

### Rendering And Viewers

Viewer and renderer APIs should be separate from physics ownership. A viewer
may synchronize with a world through explicit sync/lock/lifetime operations.
An offscreen renderer may render from an explicit world or state snapshot and
may accept caller-provided output buffers only with documented shape and
ownership rules.

Application bootstrap, windowing, rendering backends, and camera pipelines
should not leak into `World` construction before DART owns those contracts.

### Differentiable And Accelerator-Native Simulation

Differentiable simulation should expose DART-owned state/control value types,
immutable replacement or explicit commit semantics, and documented separation
between structural/static fields and dynamic arrays. Backend internals remain
hidden. Public contracts should cover shape, dtype, batch dimensions,
determinism assumptions, and verified derivative behavior.

Multiple differentiable methods can coexist as solver capabilities. The first
PLAN-110 path differentiates the existing boxed-LCP rigid-body step; a Dojo-style
path would own a separate forward solver and reverse pass, then return the same
DART-owned derivative value objects where its finite-difference gates pass.

### Custom Compute And Solver Plugins

Custom stages, solver plugins, and callback APIs require a dedicated design
covering C++ ABI stability, shared-library lifetime, exception behavior,
threading, diagnostics, and Python GIL interactions when dartpy participates.

## Deferred Capabilities

The following surfaces require public C++ owner APIs before they become part of
the DART 7 clean-break contract:

- direct loading from existing model formats into the new world;
- collision geometry, shape materials, contacts, constraints, and actuators;
- loop-closure kinematic projection and dynamic solving;
- complete rigid-body and multibody dynamics state access;
- sensors and rendering integration;
- batched worlds and accelerator-specific execution;
- custom solver plugins or compute stages that cross shared-library or Python
  callback boundaries.

Each addition should define tests, docs, examples, and API-boundary evidence
before it is considered for clean-break promotion.

## Design Rationale

- `World` stays the common entry point because it is the C++ owner of topology,
  time, frame count, stepping, and serialization in the experimental stack.
- The DART 7 experimental namespace gives maintainers room to iterate while
  preserving a clean DART 7 promotion path.
- DART 7 should remove the legacy simulation API instead of carrying two
  stable world models with conflicting ownership rules.
- Public value objects make construction, validation, binding, and docs clearer
  than exposing component structs.
- Handles give users a stable concept model while allowing ECS storage to
  remain an implementation detail.
- Explicit state/control/contact owner types are necessary before advanced
  rollout, optimization, batching, or differentiable APIs become stable.
- Kinematics-only execution belongs on the same `World` because planning,
  playback, collision checks, rendering, and sensors need the same topology,
  names, frames, and shape metadata as full physics.
- Solver names should describe methods and capabilities so the API remains
  durable even when implementations, backends, or external bridges change.
- Stable DART 7 APIs should permit implementation replacement and performance
  work behind the compatibility contract.

## DART 7 Promotion Contract

DART 7 promotes the supported DART 7 simulation API into the official C++
simulation API only after the promoted subset has:

1. public wrapper types for every user-facing concept;
2. no required includes from `comps`, `detail`, `internal`, or backend-specific
   storage;
3. focused tests for construction, stepping, state access, error handling, and
   lifetime;
4. Doxygen or user-guide docs for ownership, invalidation, and unsupported
   cases;
5. migration notes from the DART 6 simulation API to the promoted DART 7 API;
6. dartpy 7 bindings for the Python-appropriate subset;
7. release notes that state the legacy DART 6 C++ and dartpy 6 simulation APIs
   are removed.

Promotion should remove DART 7 experimental names from the recommended user
path. If temporary aliases are needed during release preparation, they should
be documented as release-scoped migration aids, not as stable DART 7
compatibility APIs.

## Verification Expectations

Docs-only edits to this design use the docs-only gate set from
`docs/ai/verification.md`.

Implementation PRs that change this API should include:

- `pixi run lint`;
- `pixi run build`;
- focused C++ tests under `tests/unit/simulation/` or the promoted
  DART 7 test path;
- `pixi run check-api-boundaries` when public headers, Doxygen scope, or dartpy
  bindings change;
- `pixi run test-py` when Python bindings are affected;
- migration notes and changelog entries when a DART 6 compatibility surface is
  deprecated, replaced, or removed.

Reviewers should reject user-facing C++ APIs that leak ECS storage, component
types, backend implementation names, or raw registry access.
