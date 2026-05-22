# Simulation Experimental C++ API

## Status

Proposal. This document owns durable API-shape rationale for the C++
`dart::simulation::experimental` surface under
`dart/simulation/experimental/**`.

DART 7 treats this API as experimental and opt-in. DART 8 is the promotion
target: the experimental simulation concepts shall become the official C++
simulation API once parity gates are satisfied, and the legacy DART 6
simulation API is removed rather than carried beside the new API.

The companion Python binding design lives in
[`simulation_experimental_python_api.md`](simulation_experimental_python_api.md).

## Purpose

The C++ experimental simulation API should give DART a clean public simulation
surface for DART 8 while allowing the DART 7 implementation to mature behind an
explicit experimental namespace. The API should expose research-facing physics
concepts and stable extension points without exposing ECS storage,
implementation components, or backend execution details.

The core design sentence is:

> The experimental C++ simulation API is the DART 7 staging surface for the
> official DART 8 simulation API, not a public contract for the underlying ECS
> implementation.

## Design Principles

### Progressive Disclosure

The common C++ path should fit in a short example:

```cpp
namespace sx = dart::simulation::experimental;

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

DART 7 treats this namespace as experimental. DART 8 should promote a stable
facade: public headers, exported symbols, Doxygen behavior, examples, dartpy
bindings, and migration notes become the compatibility contract.

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

### Tree Topology Plus Constraint Graph

Articulated systems should keep a tree-shaped `MultiBody` as the owner for
links, joints, names, and state indexing, then represent closed chains as
explicit graph constraints between symmetric public endpoints.

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

### Fresh Results Without Dirty-Flag API

The long-term API should preserve the safety of DART 6 lazy evaluation without
making the DART 6 dirty-flag network the public or required implementation.
Users should be able to set state and then read transforms, query collisions,
or render without remembering an implementation-specific cache update order.

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
from DART 7 opt-in status to the official DART 8 simulation API.

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
- DART 8 promotion and legacy API removal rules.

It does not track active implementation tasks, release priority, or migration
checklists. Those belong in `docs/plans/`, `docs/dev_tasks/`, release notes, or
the release roadmap.

## Non-Goals

- Do not make the DART 7 experimental namespace a long-term compatibility
  namespace after DART 8 promotion.
- Do not keep the legacy DART 6 simulation API beside the promoted DART 8 API.
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

- `dart::simulation::experimental::World` owns the new world lifecycle,
  topology construction, stepping, compute-executor overloads, serialization,
  and registry-backed storage.
- `World::updateKinematics()` executes the kinematics graph without the default
  rigid-body integration stage. In DART 7 this remains an explicit
  simulation-mode synchronization hook; the DART 8 target is fresh-by-default
  ordinary queries plus named synchronization hooks for predictable batching.
- Default `World::step()` composes rigid-body integration followed by
  kinematics through `WorldStepPipeline`, and pipeline overloads already allow
  selected stage execution.
- `RigidBodyOptions` already represents user-facing rigid-body initialization
  data: mass, inertia, pose, and velocity.
- `Frame`, `FreeFrame`, `FixedFrame`, `MultiBody`, `Link`, and `Joint` provide
  first-class handle concepts over the experimental storage.
- `World::getRegistry()` is a DART 7 implementation escape hatch for tests and
  internal bring-up. It is explicitly excluded from DART 8 promotion unless a
  later design creates a stable storage-inspection API.
- `StateSpace` provides a bindable, storage-independent value object for named
  flat-vector metadata.
- The experimental compute benchmark includes both world kinematics updates and
  full world stepping, which is the right evidence shape for measuring
  kinematics-only performance gains against full physics.
- Native collision already has standalone world/query concepts with explicit
  update and query operations; the experimental simulation API still needs a
  public owner bridge before those become part of the new world contract.
- The legacy DART 6-era dynamics API uses lazy forward-kinematics updates:
  transform getters compute on demand after position/velocity/acceleration
  writes mark dirty caches. That gives safe fresh reads, but it also spreads
  dirty-flag bookkeeping across frames, joints, body nodes, Jacobians, skeleton
  caches, shape caches, and support caches.
- `docs/onboarding/api-boundaries.md` requires experimental APIs to have docs,
  tests, and ownership while keeping component storage, backend plumbing, and
  implementation namespaces out of public contracts.
- `docs/onboarding/release-roadmap.md` defines DART 8 as the clean break that
  removes the legacy DART 6 API and promotes the new simulation API after
  parity gates pass.

## Namespace And Header Plan

DART 7 keeps the new simulation API under:

```cpp
namespace dart::simulation::experimental
```

and headers under:

```text
dart/simulation/experimental/**
```

DART 8 promotes the supported subset into the official stable simulation API.
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

Header families are conceptual. The exact file layout can change during DART 8
promotion, but public examples should never require implementation folders.

## Public Object Model

| Concept                | DART 7 experimental owner                                                            | DART 8 promotion target                                                                               |
| ---------------------- | ------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| `World`                | Owns topology, time, frame count, stepping, serialization, and compute entry points. | Official simulation world.                                                                            |
| `RigidBody`            | World-owned handle for a single rigid object and frame.                              | Public rigid body handle with pose, velocity, inertia, and force APIs once wrappers exist.            |
| `RigidBodyOptions`     | Public value object for mass, inertia, pose, and velocity initialization.            | Stable construction/configuration value object.                                                       |
| `MultiBody`            | World-owned handle for articulated rigid-body topology.                              | Official articulated-body concept, with final naming chosen during promotion.                         |
| `Link`                 | Body in a multibody kinematic tree and frame participant.                            | Public link handle.                                                                                   |
| `Joint`                | Connection between links with type, axes, and parent/child access.                   | Public joint handle with state/control APIs once wrappers exist.                                      |
| `LoopClosure`          | Explicit spatial closure between two public frames, links, or bodies.                | Public closed-chain handle with symmetric endpoints, diagnostics, and runtime solve policy separated. |
| `Frame`                | Spatial reference frame with transform queries.                                      | Stable frame concept for bodies, links, and user frames.                                              |
| `StateSpace`           | Named flat-vector metadata independent of storage.                                   | Stable state metadata surface for optimization and control workflows.                                 |
| Compute graph concepts | Experimental graph, executor, metadata, profile, and pipeline hooks.                 | Stable extension points only for backend-neutral concepts that pass benchmark and API-boundary gates. |

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

## Lifecycle

The C++ API keeps topology mutation separate from simulation execution:

```cpp
namespace sx = dart::simulation::experimental;

sx::World world;
auto body = world.addRigidBody("box", sx::RigidBodyOptions{});
auto robot = world.addMultiBody("arm");

world.enterSimulationMode();
world.updateKinematics();
world.step();
```

DART 7 may keep existing camelCase methods while the experimental API matures.
For DART 8 promotion, public names should follow the DART 7/8 public API naming
policy for the target namespace and headers. Compatibility wrappers for the
legacy DART 6 simulation API are not part of the stable DART 8 surface.

Design-mode errors, invalid handles, topology mismatches, and cross-world
object use should fail through documented exceptions or status-returning
functions rather than assertions in user paths.

### Closed-Chain Structure

Closed-chain mechanisms should be modeled as explicit loop-closure structure,
not as duplicated bodies, fake tree edges, or exposed internal constraint
components. The initial tree of `MultiBody`, `Link`, and `Joint` remains useful
for ownership, naming, state indexing, and articulated-body algorithms; closure
edges add graph structure on top of that tree.

The future public C++ shape should be a DART-owned handle and spec, for
example:

```cpp
auto closure = world.addLoopClosure(
    "four_bar_closure",
    sx::LoopClosureSpec{
        .frameA = groundFrame,
        .frameB = couplerLink,
        .relativeTransform = Eigen::Isometry3d::Identity(),
        .family = sx::LoopClosureFamily::Rigid,
    });

closure.setRuntimePolicy(sx::LoopClosureRuntimePolicy{
    .enabled = true,
    .kinematics = sx::ClosureKinematicsPolicy::Project,
    .dynamics = sx::ClosureDynamicsPolicy::Solve,
});
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
and the world frame. A `MultiBody` convenience method may forward to the world
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
The expected baseline is that `MultiBody::getDOFCount()` reports the
underlying tree generalized-coordinate dimension; closures add residual rows,
constraint metadata, active flags, tolerances, convergence status, and
force/impulse diagnostics. If a future API exposes independent constrained
coordinates, that should be a separate state-space view rather than silently
changing tree DOF counts.

A kinematics-only pipeline should state whether it projects closure errors,
reports residuals only, or requires the caller to select an explicit projection
stage. Dynamic closure behavior belongs to a named constraint or
implicit-dynamics stage, not to ordinary frame-cache refresh.

Diagnostics should be queryable without exposing solver rows directly. A future
diagnostic value can include the residual vector and norm, residual units and
frame convention, active/enabled state, projection or solve convergence status,
iteration count, tolerance used, and force or impulse estimates expressed in a
documented endpoint frame. If the active pipeline only reports residuals, force
and impulse fields should be absent or explicitly marked unavailable.

The API should keep the useful lessons from existing engines without inheriting
their public vocabulary:

| Engine/API pattern      | Lesson for DART's public API                                                                                     |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------- |
| MuJoCo equality models  | Keep loop closures outside the kinematic tree as named residuals that can feed dynamics.                         |
| Drake multibody plant   | Separate topology construction from finalization, and keep scalar couplers distinct from spatial closures.       |
| Project Chrono links    | Return public constraint/link handles with relative-motion, enable/disable, and reaction diagnostics.            |
| Bullet typed constraint | Support enable/disable, feedback, and solver policy without making a generic backend constraint the primary API. |

- MuJoCo models loop joints through equality constraints with residuals
  `r(q) = 0`, including connect and weld constraints outside the kinematic
  tree. That supports the DART distinction between tree joints and explicit
  closure residuals while keeping closure forces available to dynamics.
  See
  [MuJoCo computation: equality constraints](https://mujoco.readthedocs.io/en/stable/computation/index.html#equality).
- Drake exposes named multibody constraints such as ball, distance, and coupler
  constraints and locks topology at finalization. That supports design-mode
  closure creation, explicit topology validation, and state/port sizing after
  finalization. See
  [Drake MultibodyPlant](https://drake.mit.edu/pydrake/pydrake.multibody.plant.html).
- Project Chrono exposes link objects that constrain relative motion between
  bodies and can report relative displacement and force-like diagnostics. That
  supports public closure handles with diagnostic queries instead of anonymous
  solver rows. See
  [Project Chrono links](https://api.projectchrono.org/6.0.0/links.html).
- Bullet exposes a generic typed-constraint base between rigid bodies with
  enable flags, feedback, and solver-iteration overrides. That is useful
  backend evidence, but DART should expose DART-owned closure families rather
  than making a generic backend constraint object the primary API. See
  [Bullet typed constraints](https://pybullet.org/Bullet/BulletFull/classbtTypedConstraint.html).

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

For DART 7, `World::step()` and `World::step(count)` may enter simulation mode
as a common-path convenience, while `World::updateKinematics()` remains an
explicit simulation-mode synchronization hook. The current DART 7 C++ surface
still requires explicit simulation-mode entry for stepping and kinematics
updates, while dartpy currently auto-enters for `step(n)`. That mismatch is
acceptable only during the experimental stage. DART 8 promotion must make the
C++ and Python lifecycle rules identical: zero-count repeated stepping is a
no-op, positive-count stepping validates/finalizes once before the first step,
and topology mutation after finalization requires documented reset, rebuild, or
clear behavior. Repeated stepping should reuse the same executor and pipeline
state when an overload exposes them.

Rendering or application event loops should not be prerequisites for physics
stepping. Future viewer and renderer objects must own their own synchronization
and lifetime contracts.

### Kinematics-Only Runtime

The same `World` should support runtime workflows that do not integrate
dynamics. This keeps motion-planning collision checks, externally driven robot
playback, digital twins, visualization, camera or marker sensors, and
controllers on the same topology, frame, and object model as full physics.

The public distinction should be pipeline intent, not a separate world type:

```cpp
world.enterSimulationMode();
robot.setPositions(q);
world.updateKinematics();
```

A future convenience wrapper may name a kinematics-only tick or pipeline, but
the durable C++ contract is:

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

## Public Facade Rules

The promoted API should expose public wrappers before exposing implementation
fields:

- `JointType` should become a public simulation type before Python or C++ user
  examples rely on it; users should not include component headers for joint
  enum values.
- Link and joint construction should use public option/spec value objects, not
  raw component structs.
- Rigid-body pose, velocity, mass, inertia, force, torque, collision shape, and
  material access should be added through public methods before examples use
  those concepts.
- World state access should use public state views or explicit copy/write-back
  APIs, not direct registry mappers.
- Public handles must document validity after `World::clear()`, entity removal,
  and world destruction.

### Construction Value Objects

Public construction should use DART-owned value objects rather than exposing
component structs. Examples include:

- `RigidBodyOptions` for mass, inertia, pose, and velocity;
- a future `JointSpec` or equivalent for joint type, axis, limits, and parent
  relationship;
- future material/contact/geometry/appearance value objects that keep source
  geometry, physical/contact behavior, inertial data, and visualization data as
  separate concepts;
- future `WorldOptions`, `StepOptions`, or executor options for local
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
types:

- topology/model data: dimensions, names, geometry, joints, and parameters;
- state data: positions, velocities, time, caches, and solver work values;
- control data: targets, efforts, impulses, and user inputs;
- contact data: typed buffers/views produced by collision generation and
  consumed by solvers.

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

The compute graph is a valid experimental extension point, but DART 8 promotion
should include only backend-neutral concepts:

- graph nodes and explicit dependencies;
- named stage bundles for full physics, kinematics-only updates, collision
  queries, sensor updates, and rendering prep;
- sequential execution as the reference path;
- executor injection through public abstract interfaces;
- stage metadata, domain/acceleration metadata, execution profiles, and DOT
  visualization;
- world-step stages and pipelines.

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

### Custom Compute And Solver Plugins

Custom stages, solver plugins, and callback APIs require a dedicated design
covering C++ ABI stability, shared-library lifetime, exception behavior,
threading, diagnostics, and Python GIL interactions when dartpy participates.

## Deferred Capabilities

The following surfaces require public C++ owner APIs before they become part of
the DART 8 contract:

- direct loading from existing model formats into the new world;
- collision geometry, shape materials, contacts, constraints, and actuators;
- complete rigid-body and multibody dynamics state access;
- sensors and rendering integration;
- batched worlds and accelerator-specific execution;
- custom solver plugins or compute stages that cross shared-library or Python
  callback boundaries.

Each addition should define tests, docs, examples, and API-boundary evidence
before it is considered for DART 8 promotion.

## Design Rationale

- `World` stays the common entry point because it is the C++ owner of topology,
  time, frame count, stepping, and serialization in the experimental stack.
- The DART 7 experimental namespace gives maintainers room to iterate while
  preserving a clean DART 8 promotion path.
- DART 8 should remove the legacy simulation API instead of carrying two
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
- Stable DART 8 APIs should permit implementation replacement and performance
  work behind the compatibility contract.

## DART 8 Promotion Contract

DART 8 promotes the supported experimental simulation API into the official C++
simulation API only after the promoted subset has:

1. public wrapper types for every user-facing concept;
2. no required includes from `comps`, `detail`, `internal`, or backend-specific
   storage;
3. focused tests for construction, stepping, state access, error handling, and
   lifetime;
4. Doxygen or user-guide docs for ownership, invalidation, and unsupported
   cases;
5. migration notes from the DART 6 simulation API to the promoted DART 8 API;
6. dartpy 8 bindings for the Python-appropriate subset;
7. release notes that state the legacy DART 6 C++ and dartpy 6 simulation APIs
   are removed.

Promotion should remove DART 7 experimental names from the recommended user
path. If temporary aliases are needed during release preparation, they should
be documented as release-scoped migration aids, not as stable DART 8
compatibility APIs.

## Verification Expectations

Docs-only edits to this design use the docs-only gate set from
`docs/ai/verification.md`.

Implementation PRs that change this API should include:

- `pixi run lint`;
- `pixi run build`;
- focused C++ tests under `tests/unit/simulation/experimental/` or the promoted
  DART 8 test path;
- `pixi run check-api-boundaries` when public headers, Doxygen scope, or dartpy
  bindings change;
- `pixi run test-py` when Python bindings are affected;
- migration notes and changelog entries when a DART 6 compatibility surface is
  deprecated, replaced, or removed.

Reviewers should reject user-facing C++ APIs that leak ECS storage, component
types, backend implementation names, or raw registry access.
