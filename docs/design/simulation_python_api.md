# Simulation Python API

## Status

Accepted DART 7 API shape. This document owns durable API-shape rationale for
`dartpy.simulation`, the Python binding surface for `dart/simulation/**`.

DART 7 is the promotion target and clean API break. The C++ and Python
simulation APIs are the official simulation APIs for DART 7 and dartpy 7. The
legacy DART 6 C++ simulation API and legacy dartpy 6 API surface are removed
from `main` rather than carried beside the new API.

The companion C++ API design lives in
[`simulation_cpp_api.md`](simulation_cpp_api.md).

## Purpose

The Python API should make the new simulation stack usable from Python without
exposing the ECS implementation. It should be small enough for a researcher to
understand from the common-path examples, while still preserving the lower-level
handles and metadata needed for algorithm work.

The core design sentence is:

> The Python simulation API is a small, Pythonic public facade over DART's DART
> 7 simulation concepts, not a binding mirror of the C++ ECS implementation.

## Design Principles

### Progressive Disclosure

The common path should fit in a short example:

```python
from dartpy import simulation as sx

world = sx.World(time_step=0.001)
box = world.add_rigid_body(
    "box",
    mass=1.0,
    position=(0.0, 0.0, 0.5),
)

world.step(n=100)
print(world.time, box.translation)
```

Advanced users can still opt into explicit state spaces, custom compute
pipelines, and lower-level construction. Common users should not need to learn
those surfaces first.

### DART Vocabulary

Use DART and robotics vocabulary: worlds, bodies, multibodies, links, joints,
frames, DOFs, actuators, state, control, and compute stages. Avoid introducing
generic scene-graph names where DART has a clearer physics concept.

Solver, backend, preset, and example names should use algorithm, approach,
paper, or DART-owned domain names instead of other engine or project names. For
example, prefer names that describe the method, such as articulated-body,
semi-implicit integration, projected Gauss-Seidel, XPBD, or implicit time
stepping. A compatibility bridge may document what it imports or exports, but
the primary DART API name should still describe the method rather than the
originating engine.

### First-Class Objects

Creation methods return Python objects:

```python
arm = world.add_multibody("arm")
base = arm.add_link("base")
elbow = arm.joints["elbow"]
```

Users should not receive integer handles or raw component identifiers.

### Pythonic Data Access

Data-like state should be available through properties. Operations that mutate
topology, advance time, update caches, or allocate new objects should remain
methods.

```python
world.time_step = 0.001
robot.name = "arm"
print(robot.num_dofs)
world.step()
```

### Python Names Are Pythonic

The experimental Python API exposes Python spelling only. Methods, properties,
keyword arguments, and value-object fields use `snake_case`; enum members use
`UPPER_CASE` constants such as `WorldSyncStage.KINEMATICS`; class names stay
`PascalCase`, as normal Python types do.

DART 7 may keep legacy naming compatibility in legacy modules, but
`dartpy.simulation` is the staging surface for dartpy 7. It should
not bind C++ `camelCase` aliases into the experimental module. Examples,
stubs, tests, and docs should teach only the Pythonic names.

### Public Facade Before Binding Coverage

The binding should expose the useful Python concept, not every C++ method that
exists. If the only available C++ API exposes an internal type, Python waits for
a public wrapper instead of binding through the internal surface.

### Stable Facade, Replaceable Internals

DART 7 treats this module as experimental while parity gates are being closed.
Clean-break promotion should establish a stable facade: public Python names,
documented behavior, stubs, tests, and migration notes become the compatibility
contract.

That stable facade should leave DART free to change implementation details:

- add algorithms, solvers, or multi-physics stages behind existing public
  concepts;
- improve scalar CPU, SIMD, multi-core, or accelerator implementations;
- choose internal compute paths such as CUDA, Metal, Vulkan compute, ROCm/HIP,
  LLVM/JIT code generation, or future backends from benchmark evidence;
- replace storage, scheduling, collision, sensor, rendering-prep, or solver
  internals without changing user object names.

Backend and runtime names may appear in build options, diagnostic profiles, or
developer docs, but they should not define the primary dartpy API. Python users
should configure physics intent, algorithm family, accuracy/performance policy,
and fallback behavior through DART-owned value objects and capability queries.
Adding a backend should be an implementation improvement, not an API fork.

### World Scalar Precision

`sx.World()` and `sx.World(time_step=...)` remain the double-backed public path
unless a later scalar-instantiation design proves otherwise. Do not add a public
precision selector until the C++ core, bindings, stubs, serialization,
collision, differentiability, and package gates can prove that every advertised
scalar is real end to end.

Keep the API shape from becoming a one-way door. Avoid binding, stub,
serialization, and package decisions that would make later scalar support
unnecessarily invasive, while keeping the user-facing path simple and
double-backed until a dedicated scalar-instantiation design changes the public
contract.

If scalar precision becomes public later, prefer constructor or options syntax:

```python
world = sx.World(dtype=sx.float64)
```

or the equivalent `sx.WorldOptions(dtype=...)` spelling once `WorldOptions`
itself is part of the Python facade. This keeps the common construction path
Pythonic, discoverable in signatures, and aligned with array/tensor conventions.
The binding may still dispatch to concrete scalar-specific implementation
classes underneath; `dtype=` is a public factory contract, not a requirement
that nanobind represent every scalar as one runtime C++ class.

Do not make `sx.World[sx.float64]` the primary runtime construction API. Class
subscription reads like typing/generic specialization in Python and would make
precision part of public `World` identity before DART has committed to a scalar
type family. If maintainers later need scalar-specialized aliases for advanced
users or type checkers, they should be secondary to `dtype=` and covered by the
same identity, `isinstance`, stub, and migration tests.

Any future `dtype` contract must be explicit:

- default to `sx.float64` and expose a read-only `world.dtype`;
- accept only documented DART or NumPy-compatible dtype tokens;
- reject mixed-world operations and mismatched caller-provided output buffers
  unless an explicit conversion API exists;
- guarantee state, control, rollout, derivative, and bridge array dtypes; and
- reject unsupported dtypes with clear errors instead of silently computing in
  double and casting results.

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

Synchronous stepping is the reference behavior. `world.step()` must remain
deterministic, easy to test, and complete when it returns, with fresh outputs
for the stages it executed.

Asynchronous stepping may be added for throughput, UI responsiveness, server
workloads, or batched simulation, but it must compose around the same world,
pipeline, state, and synchronization concepts. Async APIs should make
ownership, completion, cancellation, and result visibility explicit. Python
`async` naming should be reserved for genuinely nonblocking or awaitable
behavior, not for a synchronous wrapper.

### Tree Topology Plus First-Class Closures

Python users should build ordinary serial chains and branched robots through
`Multibody`, `Link`, `Joint`, and `JointSpec`. Closed-chain mechanisms should
then be added as explicit first-class closure constraints between symmetric
endpoints, not by assigning a second parent link or exposing internal solver
rows.

This gives a small common path for tree-shaped robots while preserving a
natural extension for four-bars, parallel mechanisms, cable or gear couplings,
and frame-to-frame closure constraints. The closure API should separate the
topology definition from runtime choices such as enabled state, residual-only
diagnostics, kinematic projection, and dynamic constraint solving.

Closure arguments should use symmetric names such as `frame_a`/`frame_b` or
`endpoint_a`/`endpoint_b`, never `parent`/`child`. A Python user should be able
to define a closed-chain topology once, then choose whether a pipeline reports
residuals, projects kinematic state, or participates in dynamic constraint
solving without rebuilding the topology.

### Constraint Semantics Before Solver Mechanics

Python users should see semantic DART names first: loop closures, rigid
closures, point or distance closures, joint couplers, gear constraints, contact
queries, and limits. Solver options should use algorithm or approach names such
as position projection, velocity projection, Baumgarte stabilization,
compliance, damping, tolerance, maximum iterations, range-space solve,
null-space projection, or sequential impulse.

The Python API should not expose engine-named solvers or backend-named
constraint families. Model importers and diagnostics can mention source
formats or internal backends, but stable user code should configure physics
intent and algorithm family through DART-owned value objects.

### Fresh Results, Explicit Work Placement

The long-term API should not make users manually reason about dirty flags.
Users should be able to set state and then read transforms, query collisions,
or render without remembering an implementation-specific cache update order.
The overall rule is implicit freshness with explicit work placement.

The best long-term shape is a hybrid:

- common object reads are fresh by default;
- `world.step()` and kinematics-only ticks guarantee all outputs from their
  stage bundle are fresh when they return;
- explicit synchronization remains available for high-throughput loops that
  want predictable work placement;
- stale or unchecked reads are opt-in advanced behavior, not the default.

This preserves the usability of lazy evaluation without committing DART 7 to
the DART 6 dirty-flag implementation. Internally, DART may use dirty flags,
generation counters, dependency graphs, staged cache owners, or backend-specific
batch updates. The public API should expose freshness guarantees and stage
boundaries, not the cache-invalidation mechanism.

## Scope

This design covers the Python API users should see when working with
`dart/simulation/**` through dartpy.

It includes:

- module placement and import style;
- common world creation and stepping;
- first-class handles for worlds, rigid bodies, multibodies, links, joints, and
  frames;
- closed-chain kinematic and dynamic structure;
- Python naming, properties, collection views, and optional lookup behavior;
- state, control, and array semantics;
- kinematics-only execution for collision queries, kinematic state updates,
  visualization, and visual or kinematic sensors;
- compute-graph exposure boundaries;
- staged adoption rules for capabilities that require new C++ owner APIs.

It does not include implementation tracking, release priority, dashboard state,
or active task handoff. Those belong in `docs/plans/` or `docs/dev_tasks/`.

## Non-Goals

- Do not create a new top-level package. The public import remains
  `dartpy.simulation` while the API is experimental.
- Do not promote experimental symbols onto the top-level `dartpy` namespace in
  DART 7.
- Do not expose `entt::registry`, `entt::entity`, `comps`, component category
  types, raw component storage, `detail`, `internal`, backend task systems, GPU
  devices, streams, memory pools, or solver registries.
- Do not hide meaningful simulation content behind implicit world defaults.
  Examples should prefer explicit ground, gravity, and loaded model choices
  until maintainers intentionally choose a default policy.
- Do not require a full dynamics step for workflows that only need frame
  updates, collision queries, visualization, or visual/kinematic sensors.
- Do not promise differentiable simulation, tensor backends, sensors, rendering,
  model loading into the experimental world, batched environments, soft bodies,
  fluids, or Python custom compute callbacks before the C++ API provides stable
  public wrappers and focused verification.

## Current Evidence

- `dartpy.simulation` already exists as an opt-in module with
  `World`, `Multibody`, and `RigidBody` smoke coverage.
- `docs/onboarding/python-bindings.md` requires this module to remain separate
  from legacy `dartpy.simulation` during DART 7.
- `docs/onboarding/api-boundaries.md` classifies experimental APIs as public
  enough to need docs, tests, and ownership, but not public enough to expose
  storage, scheduler, backend, or component internals.
- `dart/simulation/world.hpp` already has world lifecycle,
  stepping, frame, multibody, rigid-body, and compute-executor hooks.
- The implemented DART 7 `Multibody`, `Link`, and `Joint` binding is currently
  tree-shaped, with Python-style `JointSpec` construction backed by the public
  C++ value object, joint type, axis, parent/child link access, rigid-body
  endpoint access for public rigid-body fixed joints, DOF count, and
  generalized position/velocity access. `World` now exposes `LoopClosure`
  handles with symmetric frame endpoints, semantic closure families, offsets,
  runtime participation policy, explicit residual diagnostics, lookup,
  validation, and serialization. Closure kinematic projection and dynamic
  closure solving remain staged design targets.
- The experimental dartpy facade now exposes data-like frame, joint,
  loop-closure, and rigid-body state through Python properties. Lookup and
  topology-changing operations remain methods, but parallel getter/setter-style
  aliases for those data properties are intentionally not part of the staging
  surface for dartpy 7.
- `StateSpace` is exposed as a storage-independent metadata value object in
  dartpy, with Pythonic variable names, dimensions, bounds, finalization state,
  and optional lookup by variable name. World-state extraction, write-back,
  component mapping, and rollout remain deferred until public owner APIs exist.
- DART 6-style downstream closed-chain examples use a tree skeleton plus
  solver constraints or mimic/coupler metadata. Examples such as
  `examples/rigid_loop`, `examples/coupler_constraint`, and
  `examples/mimic_pendulums` are reference material for import compatibility
  and semantics, not the DART 7/8 Python API shape.
- `world.sync(sx.WorldSyncStage.KINEMATICS)` already executes a kinematics
  graph without the default rigid-body integration stage, and
  `world.update_kinematics()` remains available in DART 7 as the existing
  synchronization spelling. C++ also has an executor overload for
  backend-neutral kinematics-only execution; dartpy should expose executor
  customization only after Python compute wrappers are deliberately promoted.
- The C++ `WorldStepPipeline` can execute selected stages, while default
  `World::step()` uses the same content-aware built-in schedule described by
  the C++ facade: solver-family and policy choices come from construction
  options or focused setters, and the default path emits only active domain
  stages before the kinematics refresh. C++ repeated-step overloads can reuse
  caller-owned executor and pipeline state; dartpy's common `world.step(n=...)`
  remains the Pythonic synchronous path until compute wrappers are promoted.
- `dart/simulation/space/state_space.hpp` provides a bindable
  value object for named state-vector metadata.
- Native collision already has standalone world/query concepts with explicit
  update and query operations; the DART 7 simulation API still needs a
  public owner bridge before exposing those from dartpy.
- The legacy DART 6-era dynamics API uses lazy forward-kinematics updates: many
  transform getters compute on demand after position/velocity/acceleration
  writes mark dirty caches. That gives safe fresh reads, but it also spreads
  dirty-flag bookkeeping across frames, joints, body nodes, Jacobians, skeleton
  caches, shape caches, and support caches.

## Layered API Shape

The public API should be layered so that each layer is independently useful:

| Layer                 | User need                                             | API stance                                                                                         |
| --------------------- | ----------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| Common world workflow | Create objects, step time, inspect results.           | `World` owns runtime state and hides model/state/control/contact separation.                       |
| Construction workflow | Build or load topology and validate it.               | Object-returning methods and value objects; no raw component structs.                              |
| Kinematic workflow    | Drive poses externally, update frames, query results. | Kinematics-only updates skip dynamics integration, constraint solves, and force accumulation.      |
| State workflow        | Copy, edit, serialize, optimize, or control state.    | Explicit state spaces, state/control values, named views, and documented copy/write-back behavior. |
| Compute workflow      | Inspect or customize stepping for research.           | Backend-neutral stage, pipeline, executor, and profile concepts only.                              |
| Internal workflow     | Storage, scheduling, component mutation, diagnostics. | Kept out of dartpy even when the C++ implementation uses those concepts.                           |

The common path stays synchronous and object-oriented. Advanced APIs may expose
functional state/control stepping later, but they should not become the first
thing a new user must understand.

## Package Surface

The DART 7 import shape is:

```python
import dartpy as dart
from dartpy import simulation as sx
```

The old `dartpy.simulation_experimental` staging module and
`DART_BUILD_SIMULATION_EXPERIMENTAL` option are retired in the clean DART 7 API.
The matching C++ surface is `dart::simulation`; the DART 6-era world surface is
not kept as a parallel Python owner.

The intended promoted public symbols are:

```text
dartpy.simulation
  World
  RigidBody
  RigidBodyOptions
  Multibody
  Link
  Joint
  JointType
  JointSpec
  WorldSyncStage
  LoopClosure
  LoopClosureFamily
  LoopClosureSpec
  LoopClosureResidual
  LoopClosureResidualCoordinates
  LoopClosureRuntimePolicy
  ClosureKinematicsPolicy
  ClosureDynamicsPolicy
  Frame
  FreeFrame
  FixedFrame
  StateSpace
  StateVariable
```

`JointSpec` is the shared C++/Python construction value object. It avoids
exposing raw link-option or component structures while still giving users a
compact way to define a parent joint.

Future submodules should be introduced only after their C++ owner APIs are
stable enough to document. The expected grouping is:

| Future group | Purpose                                                                 | Gate                                                                 |
| ------------ | ----------------------------------------------------------------------- | -------------------------------------------------------------------- |
| `state`      | State spaces, state/control values, named views, and rollout helpers.   | Public C++ state/control/contact owner types.                        |
| `compute`    | Backend-neutral stages, pipelines, executors, profiles, and graph text. | Public C++ wrappers that do not expose task-system implementation.   |
| `sensors`    | Typed sensor configs and timestamped measurement snapshots.             | Public C++ sensor owners and freshness semantics.                    |
| `rendering`  | Viewer and offscreen rendering integrations.                            | Separate viewer/render owner APIs with explicit sync/lifetime rules. |

Compute APIs belong in `dartpy.simulation.compute` only after the
public wrapper set is deliberate:

```text
dartpy.simulation.compute
  SequentialExecutor
  ParallelExecutor
  ExecutionProfile
  StageMetadata
  WorldStepPipeline
```

The Python names should avoid backend implementation names. A parallel executor
facade can exist only if the backend remains hidden.

## Common Workflow API

`World` is the primary entry point:

```python
world = sx.World(time_step=0.001)
world.time_step = 0.002
world.time
world.frame
world.is_simulation_mode
```

World configuration should be local to a `World` constructor, a documented
value object, or a future executor. The experimental API should not require a
process-wide initialization function before imports, world construction, or
stepping.

Operations stay method-shaped:

```python
world.add_rigid_body("box", mass=1.0)
world.add_multibody("arm")
world.enter_simulation_mode()
world.sync(sx.WorldSyncStage.KINEMATICS)
world.step()
world.step(n=100)
world.clear()
```

`step(n=...)` is the Python common path for repeated stepping. The
implementation may call the C++ step loop internally and should release the GIL
when no Python callback is involved. DART 7 exposes `n`; clean-break promotion
should decide whether `count` or `steps` is clearer before freezing the official
signature.

Topology mutation remains design-mode only. Stepping and kinematics updates
operate in simulation mode. Positive-count stepping may enter simulation mode
after validation; zero-count stepping is a no-op. Python and C++ should share
the same DART 7 lifecycle semantics so users do not learn different rules by
language.

The current DART 7 C++ and Python common step paths both auto-enter simulation
mode for positive-count stepping, while explicit kinematics synchronization
still requires simulation mode. The official DART 7 API should define the same
validation, auto-finalization, zero-count, and topology-mutation rules in both
languages.

### Lifecycle And Finalization

The common path should not require a mandatory build step for small examples.
`World.step()` may perform any required validation or transition into
simulation mode when the operation is deterministic and cheap enough to explain.
Explicit lifecycle methods remain important for users who need predictable
allocation, validation, or failure timing:

```python
# Future shape, not a DART 7 API promise.
world = sx.World(time_step=0.001)
robot = world.add_multibody("arm")

world.validate_topology()
world.enter_simulation_mode()
world.reset()
world.step(n=100)
```

Topology changes after simulation starts require explicit rebuild, reset, or
clear semantics. Handles returned before such a transition must either remain
valid by contract or fail with documented invalid-handle errors. Silent
reference invalidation is not acceptable for a Python-facing API.

Rendering or viewer event loops must remain separate from physics stepping. A
future viewer may have its own `sync()`, lock, close, or lifetime rules, but
those rules should not be prerequisites for `World.step()`.

### Async Stepping Shape

The synchronous `world.step()` path remains the semantic reference. A future
async API should be an explicit scheduling layer around the same stage and
state model. A true Python async surface should be awaitable, for example:

```python
# Future shape, not a DART 7 API promise.
await world.step_async(n=100)
```

If the implementation exposes a non-awaitable background job instead, the API
should use a DART-owned job or handle object with explicit completion,
cancellation, error propagation, and result-visibility rules. Backend names
belong in diagnostics and capability queries, not in public stepping methods.

### Kinematics-Only Runtime

The same `World` should support runtime workflows that do not integrate
dynamics. This matters for motion-planning collision checks, externally driven
robot playback, digital twins, visualization, camera or marker sensors, and
controllers that need fresh kinematic state without a physics solve.

The DART 7 experimental evidence already points in this direction:

- `update_kinematics()` recomputes frame caches without advancing the default
  rigid-body integration stage;
- default `step()` is the full-physics common path and may run integration,
  kinematics, and later collision/constraint/sensor stages;
- future named pipelines can run kinematics, collision queries, sensor updates,
  and rendering prep without force accumulation, integration, contact solving,
  or constraint solving.

The Python API should make this explicit instead of requiring users to discover
stage composition:

```python
world.enter_simulation_mode()
joint.position = q
world.sync(sx.WorldSyncStage.KINEMATICS)
```

`sync(...)` is a synchronization hook, not a burden every user must remember
before reading a transform. `update_kinematics()` remains a DART 7 spelling
over the same kinematics stage. Property reads and query APIs should either
ensure the needed kinematic freshness or make any advanced unchecked mode
explicit.

Future convenience names may add an explicit kinematics-only tick:

```python
# Future shape, not a DART 7 API promise.
world.step_kinematics(n=10, update_collision=True, update_sensors=True)
```

The durable contract is more important than the exact name:

- kinematics-only updates never integrate velocities or forces;
- they do not solve contacts or constraints unless a named query stage asks for
  collision detection;
- they may update time/frame counters only through documented tick semantics;
- collision, visualization, and visual/kinematic sensors consume the same
  frame caches as full physics;
- common reads stay fresh by default, while advanced bulk loops can call the
  explicit synchronization hook once before many reads;
- benchmarks compare kinematics-only pipelines against full physics pipelines
  for the same scene before advertising performance gains.

### Freshness And Cache Semantics

Freshness is part of the user contract:

```python
joint.position = q
tool_pose = robot.links["tool"].transform  # Fresh by default.
```

The API should define these rules:

- state writes invalidate derived kinematics, collision, rendering, and sensor
  data as needed;
- `joint.position` and `joint.velocity` are vector properties with length
  `joint.num_dofs`; position writes drive open-chain forward-kinematics
  refreshes for standard tree joints, while closed-chain projection remains a
  staged solver capability;
- ordinary object properties and query methods produce fresh results by
  default, even if that triggers internal synchronization;
- bulk workloads can call `world.sync(sx.WorldSyncStage.KINEMATICS)` or the
  DART 7 `world.update_kinematics()` spelling once, then read many values
  without repeated hidden work;
- advanced unchecked reads, if added, must be visibly named and documented as
  possibly stale;
- query APIs that can be expensive should support an explicit update policy
  once the C++ owner APIs exist.

One possible future query shape is:

```python
# Future shape, not a DART 7 API promise.
world.sync(sx.WorldSyncStage.KINEMATICS)
poses = robot.links.transforms
contacts = world.collision.query(update=sx.UpdatePolicy.IF_STALE)
```

The exact names can change, but the principle should not: easy code gets fresh
answers; performance-sensitive code gets explicit batching and profiling; no
public API exposes dirty flags.

## Public Object Model

| Object        | Role                                                                  | Initial Python shape                                                                                                                           |
| ------------- | --------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `World`       | Owns simulation objects, time, frame count, and stepping.             | Constructor, lifecycle methods, time properties, add methods, object collections.                                                              |
| `RigidBody`   | Single rigid object and frame handle.                                 | Name, transform, velocity, mass, inertia, force, and torque reads/writes, and broader dynamics properties as accessors mature.                 |
| `Multibody`   | Articulated rigid-body system.                                        | Name, validity, counts, link and joint construction, link and joint collections.                                                               |
| `Link`        | Body in a multibody kinematic tree.                                   | Name, parent joint, frame transform queries.                                                                                                   |
| `Joint`       | Connection between links or public rigid-body fixed-joint endpoints.  | Name, type, axes, link or rigid-body endpoint handles, DOF count, generalized position and velocity; broader state/control APIs remain staged. |
| `LoopClosure` | Explicit spatial closure between two public frames, links, or bodies. | Symmetric-endpoint topology handle with runtime-intent policy and residual diagnostics now; projection and dynamic solving remain staged.      |
| `Frame`       | Spatial reference frame.                                              | Transform, translation, rotation, quaternion, parent-frame queries.                                                                            |
| `StateSpace`  | Named flat-vector metadata.                                           | Variables, dimensions, bounds, finalization, names.                                                                                            |

`Multibody` is the experimental name because "multibody system" is the standard
term in the multibody-dynamics literature (Featherstone, _Rigid Body Dynamics
Algorithms_; Shabana) and matches peer robotics-dynamics libraries (Drake's
`MultibodyPlant`). It is spelled as one word to follow that literature. The
engine-specific term "Articulation" (PhysX/Isaac/Newton) was considered and
rejected because it does not appear in the robotics/dynamics literature as the
name of the object. Examples should teach `Multibody` and make the relationship
to links and joints obvious.

Creation methods return objects owned by a world or by another public object.
This is a future target shape once fixed rigid bodies, link construction
options, and collection owners are fully bound:

```python
world = sx.World()
ground = world.add_rigid_body("ground", fixed=True)
arm = world.add_multibody("arm")
base = arm.add_link("base")
tool = arm.add_link("tool", parent=base)
```

Object references are the primary runtime control surface. Serialized names,
resource paths, import diagnostics, and visualization paths are useful metadata,
but they should not become the common way to command physics objects.

### Composition Model

The public model should expose physics concepts, not storage entities:

- a `World` owns time, topology, state, and collections;
- a `Multibody` owns links, joints, actuators, and state dimensions;
- a `RigidBody` owns a body frame and rigid-body properties;
- a `Link` participates in a kinematic tree and owns or references a frame;
- a `Joint` connects parent and child links and owns state/control dimensions;
- a `LoopClosure` closes a spatial loop between two public frames, links, or
  bodies without exposing solver rows or ECS components;
- future joint couplers, gear constraints, and scalar multibody constraints
  should use their own family or a broader multibody-constraint umbrella rather
  than overloading `LoopClosure`;
- future sensors attach to frames, rigid bodies, links, joints, or named
  selections through public handles;
- future geometry APIs separate source geometry, physical/contact properties,
  inertial properties, and visual appearance.

Soft bodies, fluids, deformables, and terrain can be added later using the same
first-class-object rule. They should not force the initial rigid-body API to
adopt generic entity names or ECS vocabulary.

## Properties And Methods

Use properties for data-like reads and writes:

```python
world.time_step
world.time
world.frame

body.name
body.transform
body.translation
body.rotation
body.quaternion

robot.name
robot.num_links
robot.num_joints
robot.num_dofs

joint.name
joint.type
joint.axis
```

Use methods for operations:

```python
world.step()
world.clear()
world.sync(sx.WorldSyncStage.KINEMATICS)
tool.relative_transform(base)
robot.add_link("base")
body.apply_force((1.0, 0.0, 0.0))
```

Getter-style C++ methods may exist underneath, but the experimental Python
binding should expose native `snake_case` methods only for real operations and
lookups, and should expose data-like state through properties. Runtime
camelCase compatibility and parallel data getter/setter aliases belong to
legacy modules, not to the DART 7 experimental staging surface for dartpy 7.

## Collections And Lookup

Collections should become the normal access path once public C++ enumeration
wrappers and name-uniqueness policy exist:

```python
len(world.multibodies)
world.multibodies.names
world.multibodies.get("arm")      # Optional lookup, returns None if missing.
world.multibodies["arm"]          # Required lookup, raises KeyError if missing.

len(robot.links)
robot.links.names
robot.links["base"]

len(robot.joints)
robot.joints.names
robot.joints["elbow"]
```

Named objects should be unique within their public owner before dict-style
collection access is promoted. `World` owns unique multibody, rigid-body, and
loop-closure names; `Multibody` owns unique link and joint names. Autogenerated
names should skip existing names so explicit names and generated names cannot
silently collide. Optional lookup methods such as `get_multibody()` and
`get_link()` may continue to return `None` for missing names during DART 7
staging. Presence methods such as `has_multibody()`, `has_rigid_body()`, and
`has_loop_closure()` are Pythonic operations for checking world-owned names
without promoting a full dict-style collection object yet.

The DART 7 experimental binding may stage construction-ordered snapshot lists
for owner-local collections whose ordering is already explicit in the public
C++ owner, such as `robot.links`, `robot.joints`, `robot.link_names`, and
`robot.joint_names`. These properties should remain list-like until the full
collection contract below is documented.

The collection contract should define:

- whether names must be unique at construction time;
- whether autogenerated names are stable and user-visible;
- whether `.get(name)` returns `None`, raises, or accepts a default;
- whether `collection[name]` raises `KeyError` for missing names;
- whether `.names` preserves construction order;
- whether lookup is precomputed and stable across simulation steps;
- how removal, rebuild, or `World.clear()` affects collection views.

Bulk properties belong on collections only when the C++ API provides a stable
state owner:

```python
robot.joints.position
robot.joints.velocity
robot.joints.target_position = targets
```

Until then, the design records this as an API goal rather than a binding of
internal component fields.

## Closed-Chain Structure

Closed-chain mechanisms should be first-class API concepts, not workarounds
that duplicate links, add fake tree edges, or expose internal constraint
components. The tree structure of a `Multibody` remains the owner for names,
links, joints, state indexing, and articulated-body algorithms. Loop closures
add graph edges on top of that tree.

DART 7 now stages the topology, runtime-intent, and residual-diagnostic part
of this model:
`world.add_loop_closure(...)` returns a first-class `LoopClosure` handle with
symmetric endpoint frames, optional auto-naming, semantic family, endpoint
offsets, name lookup, count queries, validation, serialization, direct runtime
participation properties, and a Pythonic `LoopClosureRuntimePolicy` value
object for batch assignment. `closure.compute_residual()` returns explicit
closed-chain residual diagnostics without exposing solver rows. Constrained
kinematic projection and dynamic solving remain clean-break target concepts to
stage behind the experimental module before promotion. Active projection or solve
policies are rejected at runtime until compatible stages exist, while disabled
closures may retain future-intent policy metadata.

The staged Python shape uses compact value objects or keyword construction and
returned public handles:

```python
closure = world.add_loop_closure(
    "four_bar_closure",
    frame_a=ground_frame,
    frame_b=coupler,
    offset_a=(
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ),
    family=sx.LoopClosureFamily.RIGID,
)
```

Runtime policy is public metadata while projection and solving remain staged:

```python
closure.enabled = True
closure.kinematics = sx.ClosureKinematicsPolicy.PROJECT
closure.dynamics = sx.ClosureDynamicsPolicy.SOLVE
```

Residual diagnostics are explicit queries rather than implicit properties:

```python
residual = closure.compute_residual()
```

The minimal world-owned construction surface should be:

```python
spec = sx.LoopClosureSpec(frame_a=ground_frame, frame_b=coupler)
closure = world.add_loop_closure("four_bar_closure", spec)
closure = world.get_loop_closure("four_bar_closure")
exists = world.has_loop_closure("four_bar_closure")
count = world.num_loop_closures
```

The exact names can change before promotion, but the API should distinguish:

- symmetric closure endpoints from tree parent/child relationships;
- closure topology from runtime participation policy;
- kinematic projection, residual-only diagnostics, and full dynamic constraint
  solving as selectable runtime behavior;
- topology validation and finalization from per-step runtime activation;
- spatial closure families such as rigid, point, distance, and axis closures;
- scalar couplers and gear constraints from spatial loop closures;
- public tolerance, stabilization, enable/disable, and diagnostic fields
  without exposing backend solver rows.

Closed-chain APIs must define ownership and lifetime. World-owned closures are
the conservative default because endpoints may span multibodies, rigid bodies,
and the world frame. A `Multibody.add_loop_closure(...)` convenience can
forward to the world only when both endpoints are in the same owner. Cross-world
endpoints must be rejected, and removal, serialization, topology rebuilds, and
`world.clear()` must document handle invalidation.

Closed-chain APIs must define how closures affect DOF counts, state-space
metadata, serialization, collision filtering, handle validity, and residual
reporting. `Multibody.num_dofs` should continue to mean the underlying tree
coordinate dimension; closures add residual rows, active flags, tolerances,
convergence status, and force/impulse diagnostics. The current
`LoopClosureResidual` value reports residual vectors and norms, world-frame
coordinate convention, active/enabled state, and force availability. Future
diagnostics can add projection or solve convergence, tolerance used, and solved
force/impulse estimates. A kinematics-only pipeline should state whether it
projects closure errors, reports residuals only, or requires an explicit
projection stage. Dynamic closure behavior belongs to a named constraint or
implicit-dynamics stage, not to ordinary frame-cache refresh.

Existing engine APIs point to the same broad shape:

| Engine/API pattern     | Python-facing lesson                                                                                          |
| ---------------------- | ------------------------------------------------------------------------------------------------------------- |
| MuJoCo equality models | Keep loop closures outside the tree as named residuals, with closure forces available only when solved.       |
| Drake multibody plant  | Use named spatial constraints and make them reusable by kinematics-only optimization/projection workflows.    |
| PhysX articulations    | Preserve tree articulations and close loops with constraints, while documenting approximation limits.         |
| Isaac Sim rigging      | Importers may cut a closed CAD loop and add couplers, but user code should see closures and couplers.         |
| Gazebo/SDFormat graphs | Graph model formats need explicit runtime validation, cut-joint, and residual policy in the API.              |
| PyBullet facade        | A stable Python facade can hide engine replacement, but DART should use typed objects instead of integer IDs. |
| CoppeliaSim routines   | Kinematics-only loop solving should be first-class, separate from dynamic stepping.                           |

The key Python lesson is that `add_loop_closure(...)` should look like normal
object construction, while lower-level residual rows, solver backend data,
integer constraint IDs, cut-joint heuristics, and maximal-coordinate
implementation choices stay behind the C++ facade. The companion C++ design
records the source links and lower-level engine constraints.

## Programmatic Construction

The Python construction API should use compact value objects or keyword
arguments instead of raw C++ option structs when those structs expose internal
component vocabulary.

```python
world = sx.World()
arm = world.add_multibody("arm")

base = arm.add_link("base")
forearm = arm.add_link(
    "forearm",
    parent=base,
    joint=sx.JointSpec(
        name="elbow",
        type=sx.JointType.REVOLUTE,
        axis=(0.0, 0.0, 1.0),
    ),
)

world.sync(sx.WorldSyncStage.KINEMATICS)
```

`RigidBodyOptions` is a good initial value object because its public fields
already map to user concepts: mass, inertia, pose, and velocity.

```python
box = world.add_rigid_body(
    "box",
    sx.RigidBodyOptions(
        mass=1.0,
        position=(0.0, 0.0, 0.5),
        linear_velocity=(0.0, 0.0, 0.0),
    ),
)
```

Convenience keyword arguments may be supported by constructing
`RigidBodyOptions` internally, but the options object should remain available
for explicit workflows.

### Configuration Value Objects

Configuration should use validated value objects for stable concepts and
keyword shortcuts for the common path. Examples:

```python
world = sx.World(
    time_step=0.001,
    gravity=(0.0, 0.0, -9.81),
    rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
    multibody_options=sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.SEMI_IMPLICIT
    ),
    contact_solver_method=sx.ContactSolverMethod.SEQUENTIAL_IMPULSE,
    contact_gradient_mode=sx.ContactGradientMode.ANALYTIC,
)

body = world.add_rigid_body(
    "box",
    sx.RigidBodyOptions(
        mass=1.0,
        position=(0.0, 0.0, 0.5),
    ),
)
```

Options for simulation, rigid-body construction, joint construction, sensors,
rendering, and execution should stay in separate value objects. Renderer,
camera, GPU, and sensor-pipeline options should not leak into `World` until
DART owns those subsystems.

The current dartpy binding exposes World-level solver defaults and policies as
constructor keywords rather than a bound `sx.WorldOptions` object. If Python
later gains `sx.WorldOptions`, it should preserve the same field names and
validation behavior as those constructor keywords.

### Loading And Source Geometry

C++ now owns the first parsed-Skeleton bridge for the tree-joint families that
map to the experimental multibody facade (Weld, Revolute, Prismatic, Screw,
Universal, Ball, Planar, and Free), and dartpy exposes the same already-parsed
bridge as `sx.add_skeleton(world, skeleton, options=...)` with
`SkeletonLoadOptions`. The same Python function now accepts URI strings through
the C++ `dart::io::readSkeleton()` reader path, while
`sx.add_world(world, source_world_or_uri, options=...)` applies the same importer
to every Skeleton in an already-parsed or URI-loaded legacy World. URI-loading
overloads accept `sx.ReadOptions` for explicit format selection, SDF default
root-joint selection, and URDF package directories. The bridge also imports one
centered collidable Box/Sphere/Capsule/Cylinder/Mesh collision shape per link
when the legacy shape maps exactly to the experimental `CollisionShape` facade;
multiple
collision shapes, source offsets, visual geometry, and material data remain
deferred rather than approximated. Resource retriever bindings,
unsupported-feature diagnostics, and the richer load-result shape remain
deferred. The eventual shape should preserve these rules:

- file loading returns first-class public objects or a structured load result;
- imported names and source paths are metadata, not the primary control API;
- geometry/source description, physical material/contact behavior, inertial
  properties, and visual appearance remain separate concepts;
- load warnings and unsupported features are structured diagnostics;
- topology mutation after loading follows the same design-mode lifecycle as
  programmatic construction.

## State, Control, Contacts, And Arrays

Python state access should be explicit about ownership:

- scalar and small transform properties may return Python scalars or NumPy
  arrays;
- arrays returned from transforms, positions, velocities, and state spaces
  should be copies unless the API explicitly documents view lifetime and
  mutation semantics;
- mutable state writes should be property assignments or explicit write-back
  transactions, not silent mutation of exposed ECS storage;
- state-vector APIs should start from `StateSpace` metadata and a future
  public `WorldStateView`, not from raw registry mappers.
- `World.state_vector` / `World.control_vector` are the general dense world
  vectors: dynamic rigid-body translation coordinates first, then multibody
  joint generalized coordinates in baked dense order. The previous
  translational rigid-body reduction is explicit as
  `World.rigid_body_state_vector` / `World.rigid_body_control_vector`, with
  `num_rigid_body_dofs` / `num_rigid_body_efforts` reporting that scoped
  dimension.

Advanced APIs should distinguish at least four concepts:

| Concept      | Meaning                                            | Common-path visibility                                        |
| ------------ | -------------------------------------------------- | ------------------------------------------------------------- |
| Topology     | Bodies, links, joints, geometry, dimensions.       | Hidden behind `World`, `RigidBody`, and `Multibody` objects.  |
| State        | Time-varying positions, velocities, and caches.    | Owned by `World.step()` until explicit state APIs are used.   |
| Control      | Commands, targets, efforts, and inputs.            | Object or collection properties first, explicit values later. |
| Contact data | Collision results and constraint data for solvers. | Deferred until public contact buffers/views exist.            |

Replay and scrubber APIs should expose recording controls, frame metadata, and
explicit restore calls, not raw component snapshots. Per-frame replay storage is
runtime-state-only: time/frame metadata, active solver mode, positions,
velocities, forces, controls, diagnostics, and solver work values that affect
continuation. Topology, geometry, materials, assets, and static construction
options are not duplicated per frame; if they change after recording, restore
should reject the frame as layout-incompatible.

A future state-view workflow should look like:

```python
space = sx.StateSpace()
space.add_variable("arm.q", dimension=6)
space.add_variable("arm.dq", dimension=6)
space.finalize()

state = world.get_state(space)
state["arm.q"] = q
world.set_state(state)
```

The exact view type can change, but the invariant is stable: Python state
objects do not expose component storage or registry access.

Future control and contact APIs should follow the same pattern:

- control values are explicit objects or named arrays with documented shape;
- contact data is a typed public buffer or view produced by collision
  generation and consumed by solvers;
- optional contact attributes are requested before allocation when the choice
  changes memory layout or performance;
- copies, views, and write-back behavior are documented per field.

### Advanced State And Rollout Shape

A future rollout API should be separate from normal world stepping:

```python
trajectory = sx.state.rollout(
    world.model(),
    initial_state=state,
    control=control_sequence,
    steps=200,
    out=optional_output_buffer,
)
```

The names above are illustrative, not DART 7 API promises. The durable design
constraints are:

- rollout is functional or stateless from the caller's perspective;
- initial state, control, and output arrays have documented batch and step
  dimensions;
- optional caller-provided output buffers have explicit shape and dtype checks;
- worker pools, devices, streams, kernels, and allocator details remain hidden;
- the common `World.step()` path stays simpler than the rollout path.

## Compute Surface

The compute graph is an important research-facing extension point, but the
Python API should expose only backend-neutral concepts.

Allowed public concepts after wrapper review:

- graph nodes and explicit dependencies;
- named stage bundles for full physics, kinematics-only updates, collision
  queries, sensor updates, and rendering prep;
- sequential execution as the reference path;
- a backend-hidden parallel executor facade;
- stage metadata and execution profiles;
- DOT or text visualization of graph structure;
- world-step stages and pipelines.

Implementation backends such as scalar CPU, SIMD, thread pools, task graphs,
CUDA, Metal, Vulkan compute, ROCm/HIP, or LLVM/JIT code generation may improve
performance behind this boundary. They should not appear as required public
types, module names, solver names, or object identities unless a later design
promotes a backend API intentionally.

Not allowed in Python public API:

- backend implementation names;
- raw task-graph types;
- GPU devices, streams, kernels, memory pools, or transfer queues;
- scheduler resource ownership contracts before resource metadata has matured
  beyond diagnostics;
- C++ callback entry points that can execute Python code in parallel without a
  documented GIL and lifetime policy.

## Solver And Execution Policy

Solver and execution names should describe algorithms, numerical methods, or
DART-owned policies. They should not be named after external engines,
runtimes, or projects.

The public solver documentation should use a capability matrix instead of
backend names:

| Capability         | Examples of documented values                                 |
| ------------------ | ------------------------------------------------------------- |
| Integration family | semi-implicit Euler, implicit Euler, variational integrator.  |
| Dynamics approach  | articulated-body method, constrained dynamics, XPBD.          |
| Constraint solve   | projected Gauss-Seidel, direct solve, barrier method.         |
| Coordinate support | maximal coordinates, generalized coordinates, mixed systems.  |
| Supported features | contacts, joints, actuators, soft constraints, closed chains. |
| Execution shape    | single world, kinematics-only, homogeneous batch, rollout.    |
| Differentiability  | unsupported, finite-difference checked, analytic, autodiff.   |

The DART 7 Python API should not expose solver registries, plugin loaders, or
backend-specific solver names. Explicit solver and contact-policy selection uses
DART-owned capability names on the `World` constructor and focused value objects
such as `MultibodyOptions`.

New solvers and multi-physics stages should be additive under DART-owned
capability names. A user should be able to ask for a method family or policy
and receive a documented fallback or unsupported-capability error when the
current build lacks the required implementation backend.

The current binding mirrors the C++ construction-time grouping without exposing
solver internals:
`sx.World(rigid_body_solver=..., multibody_options=..., contact_solver_method=..., contact_gradient_mode=...)`
sets the built-in schedule defaults and policies up front, while the
`rigid_body_solver`, `multibody_options`, and `contact_gradient_mode` properties
remain available for interactive configuration when they are safe to switch
after construction. Invalid method-family names and enum values are rejected
before the world starts stepping.

## Future Capability Shapes

These sections describe target shapes, not runnable DART 7 APIs.

### Kinematic Queries And Collision

Kinematics-only workflows should support query stages that run after frame
updates and before optional visualization or sensor updates:

```python
# Future shape, not a DART 7 API promise.
robot.joints.position = q
world.sync(sx.WorldSyncStage.KINEMATICS)
contacts = world.collision.query()
visible = world.visibility.query(camera)
```

The query API should be clear about what it updates:

- frame caches and shape transforms are refreshed before query results are
  produced;
- collision or distance queries may update broad-phase data structures, but do
  not solve contact impulses;
- visual queries and visual sensors use renderable geometry or visibility
  data, not dynamics state;
- kinematic sensors read frame transforms, twists, marker positions, or
  distances and do not require forces, masses, or integrators;
- stale-query behavior is explicit when users mutate poses without updating
  kinematics.

### Sensors

Future sensors should follow a create-configure-attach-read lifecycle:

```python
# Future shape, not a DART 7 API promise.
imu = world.sensors.add_imu(
    "imu",
    attach_to=robot.links["tool"],
    update_period=0.001,
)
world.reset()
world.step()
sample = imu.read()
```

The eventual API should define attachment targets, required state/contact
attributes, reset behavior, update cadence, timestamps, units, and stale-data
semantics. Sensor snapshots should be typed values, not raw dicts of internal
buffers.

Sensor docs should classify whether a sensor needs full physics, collision
query data, rendering data, or kinematics only. Visual and kinematic sensors
should be usable from a kinematics-only pipeline when their inputs are
available.

### Rendering And Viewers

Rendering should remain opt-in and separate from physics ownership. A future
interactive viewer may synchronize with a world, while an offscreen renderer may
render from an explicit world or state snapshot. Both surfaces need clear
lifetime, thread, sync, and close rules before becoming public.

Viewer state should not be stored as hidden physics state. Offscreen rendering
should allow caller-provided output arrays only when shapes, dtypes, and
ownership are documented.

### Batched Worlds

Batched execution is deferred, but the contract should be planned early:

- homogeneous replication is the first target;
- arrays use an explicit leading world/environment dimension;
- per-world options are explicit value objects or named arrays;
- environment selection uses explicit indices or masks;
- visual spacing is separate from physics coordinates;
- heterogeneous worlds and dynamic topology changes are later capabilities;
- tensor backend details remain hidden behind DART-owned facades.

### Differentiable Simulation

Differentiable or accelerator-native simulation should live in a future optional
submodule or package. The intended shape is conversion into immutable state
values, explicit replacement/write-back operations, and documented separation
between structural/static fields and dynamic arrays.

Do not expose accelerator internals in dartpy. The public contract should be
state/control shape, dtype, batch semantics, determinism assumptions, and
verified derivative behavior.

### Custom Compute And Solver Plugins

Python-defined compute stages, solver callbacks, and plugin loaders require a
separate design. They need C++ ABI rules, Python lifetime rules, exception
mapping, thread-safety, GIL behavior, and benchmark evidence before they become
public API.

## Deferred Capabilities

The following ideas are valuable but should be documented as future work until
they have public C++ owner APIs and objective-specific verification:

- file loading directly into the experimental world;
- collision geometry, shape materials, contacts, constraints, and actuators;
- loop-closure kinematic projection and dynamic solving;
- rigid-body collision coupling and broader pose/state accessors beyond the
  currently public transform, velocity, mass, inertia, force, and torque
  wrapper set;
- multibody dynamics state, joint limits, effort limits, and control commands;
- sensors and the create-attach-read lifecycle;
- rendering and viewer integration;
- differentiable simulation and tensor backends;
- batched environments and accelerator-specific execution;
- Python-defined compute stages and solver plugins.

Deferring these surfaces is not a rejection of the design direction. It keeps
the Python API honest about what the C++ experimental module owns today.

## Comparison to Existing APIs

This section compares API patterns, not products. The goal is to make DART's
tradeoffs explicit without judging other projects or importing their vocabulary
as DART policy.

| Pattern                              | Strength                                                                                          | DART experimental Python choice                                                                                                                |
| ------------------------------------ | ------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| Current dartpy direct bindings       | Keeps DART concepts visible and gives existing users continuity.                                  | Keep `dartpy` as the package and keep DART terminology, but present new experimental bindings through Pythonic properties and focused facades. |
| Functional model/state stepping      | Makes data ownership explicit and supports advanced rollout control.                              | Hide model/state separation in the common `World.step()` path, then expose explicit state spaces and state views for advanced workflows.       |
| Compiled topology plus runtime state | Allows multiple states, reset points, and advanced sampling.                                      | Keep topology and simulation state separate internally; expose lifecycle and state views only through public DART handles.                     |
| Procedural model editing             | Gives programmatic construction a clear validation point.                                         | Treat topology mutation as design-mode work and require explicit reset/rebuild semantics after simulation starts.                              |
| Object-returning construction        | Lets users keep direct references to the objects they create.                                     | Return `World`, `RigidBody`, `Multibody`, `Link`, and `Joint` objects rather than integer handles or component identifiers.                    |
| Typed named collections              | Gives convenient lookup while preserving grouped metadata.                                        | Add `.names`, `.get()`, and `[]` only after uniqueness, ordering, missing-name, and invalidation behavior are documented.                      |
| Path-based scene addressing          | Works well for serialization and rendering hierarchies.                                           | Keep paths and serialized names out of the runtime control API; use Python objects and named collections.                                      |
| Grouped option objects               | Keeps configuration structured and validates fields early.                                        | Use DART value objects for stable configuration, with keyword shortcuts for common construction paths.                                         |
| Geometry/material/appearance split   | Separates source data, physics behavior, and visualization.                                       | Keep source geometry, physical/contact properties, inertial properties, and visual appearance as separate future concepts.                     |
| Mandatory build/finalize lifecycle   | Makes allocation and compilation points explicit.                                                 | Keep explicit lifecycle hooks available only when needed; the common path should make `World.step()` the obvious operation.                    |
| Global initialization                | Centralizes backend configuration.                                                                | Avoid global mutable setup for DART 7 simulation; configure a `World` or future executor object directly.                                      |
| Stateless rollout                    | Supports batched sampling without mutating a live world.                                          | Add rollout only after explicit state/control/contact owner APIs exist; keep it separate from `World.step()`.                                  |
| Kinematics-only execution            | Supports planning, playback, collision queries, visualization, and sensors without dynamics cost. | Use the same `World` with an explicit kinematics/query pipeline that skips integration and constraint solving.                                 |
| Bulk articulation views              | Supports vectorized control and inspection.                                                       | Add collection-level DOF views only after C++ defines stable state owners, shape rules, and invalidation behavior.                             |
| Sensor create-attach-read lifecycle  | Gives sensors clear freshness and ownership rules.                                                | Defer sensors, but require typed configs, public attachment targets, world reset/update integration, and timestamped snapshots.                |
| Viewer synchronization               | Keeps UI/render state coherent with physics state.                                                | Keep viewers/renderers separate from physics ownership, with explicit sync, locking, lifetime, and output-buffer contracts.                    |
| Batched leading dimension            | Makes replicated workloads explicit in array shapes.                                              | Plan future batched worlds around a leading world dimension and explicit selection, without exposing backend runtime details.                  |
| Backend-specific acceleration APIs   | Can expose advanced performance controls early.                                                   | Keep public concepts backend-neutral until benchmark and packaging evidence justify a stable accelerator contract.                             |

## Design Rationale

- `World` is the common entry point because it is DART vocabulary and already
  owns time, topology, and stepping in the experimental C++ API.
- The API avoids global initialization because simulation configuration should
  be local, testable, and composable across multiple worlds in one process.
- Construction methods return objects because object references are clearer
  than integer IDs and avoid exposing component identifiers.
- Properties are preferred for data-like reads and writes because they match
  Python expectations; methods remain for operations that allocate, mutate
  topology, update caches, or advance time.
- Named collections are a goal, but they wait for an explicit uniqueness and
  invalidation policy.
- The model/state/control split is hidden in the common path to keep examples
  small, then exposed deliberately for optimization, rollout, and control
  workflows.
- Kinematics-only execution belongs on the same `World` because planning,
  playback, collision checks, rendering, and sensors need the same topology,
  names, frames, and shape metadata as full physics.
- The public facade stays stable after DART 7 promotion so DART can improve
  algorithms, solvers, multi-physics stages, and compute backends without
  forcing user code to track implementation changes.
- Paths and source names are metadata for files, diagnostics, and rendering;
  they are not the primary runtime control API.
- Sensors, rendering, batched execution, differentiability, and custom solvers
  are designed as future modules so the initial API can remain honest about
  what DART owns today.

## Migration And Promotion Path

DART 7 keeps the experimental Python surface under
`dartpy.simulation`. The module can grow while its contracts are
still marked experimental, but growth should follow these promotion rules:

1. Expose only public C++ wrapper types or Python facade types backed by public
   C++ accessors.
2. Add focused Python tests for imports, construction, property behavior,
   errors, lifetime, and stubs.
3. Update committed stubs with every public binding addition.
4. Run API-boundary checks when bindings reach into new C++ headers.
5. Document unsupported cases and future promotion conditions.
6. Promote for DART 7 and dartpy 7 only after parity gates show that the
   experimental world can replace the classic world for the supported
   workflow.

DART 7 and dartpy 7 make this surface official and remove the legacy DART 6
C++ and dartpy APIs from the stable path. The promoted API should preserve
the design invariants in this document: small common path, first-class objects,
Pythonic properties, explicit advanced state, and no ECS leakage.

## Quick Reference

In these snippets, `sx` is shorthand for the simulation API module. During
parity work that module is `dartpy.simulation`; after DART 7
promotion it is the official dartpy 7 simulation path chosen during promotion.

DART 7 experimental design target for the common path:

```python
from dartpy import simulation as sx

world = sx.World(time_step=0.001)
box = world.add_rigid_body("box", mass=1.0, position=(0.0, 0.0, 0.5))
world.step(n=100)
print(world.time, box.translation)
```

DART 7 experimental design target for construction:

```python
arm = world.add_multibody("arm")
base = arm.add_link("base")
forearm = arm.add_link(
    "forearm",
    parent=base,
    joint=sx.JointSpec(
        name="elbow",
        type=sx.JointType.REVOLUTE,
        axis=(0.0, 0.0, 1.0),
    ),
)
world.sync(sx.WorldSyncStage.KINEMATICS)
```

DART 7 promoted target after owner APIs exist:

```python
space = sx.state.StateSpace()
space.add_variable("arm.q", dimension=6)
space.add_variable("arm.dq", dimension=6)
space.finalize()

state = world.get_state(space)
control = world.get_control(space)
trajectory = sx.state.rollout(world.model(), state, control, steps=200)
```

The promoted target example is intentionally separated from the current DART 7
experimental examples.
It records the design direction for state/control/rollout APIs without
pretending those symbols are already supported by the experimental bindings.

## Verification Expectations

Docs-only edits to this design use the docs-only gate set from
`docs/ai/verification.md`.

Implementation PRs that change this API should include:

- focused `python/tests` coverage for the new surface;
- committed stub updates under `python/stubs/dartpy/`;
- `pixi run lint`;
- `pixi run build` for binding changes;
- `pixi run test-py`;
- `pixi run lint-api-boundaries` when new C++ headers enter dartpy;
- focused C++ tests when new public wrapper APIs are added under
  `dart/simulation/**`.

Reviewers should reject bindings that expose internal ECS storage, backend
implementation names, or raw component access even when the surrounding C++
module is experimental.
