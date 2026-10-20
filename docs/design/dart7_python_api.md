# DART 7 Python-first API and core contracts

This document owns the durable rationale and rules for the DART 7.0 Python API
(`dartpy`), and for the core-engine contracts that API relies on. Active
sequencing, packets and gates live in the
[DART 7.0 project home](../dev_tasks/dart7_python_first/README.md) and the
[plan dashboard](../plans/dashboard.md).

## Goals and boundary

DART 7.0 makes DART Python-first. The Python API is the primary user surface and
is designed for robot learning, robotics and control research,
character/biomechanics and soft-body research, and education alike. Conflicts
between those audiences are resolved by layering, not by dropping one of them.

The C++ API stays source- and behavior-compatible for the subset that
gz-physics and gz-sim use. Other C++ APIs are best effort: they may change,
but each public-API or ABI-layout change needs maintainer approval. No
deprecated C++ API is removed during 7.x.

DART 7.0 is a **clean break for Python**:
- there are no runtime aliases or deprecation shims for the 6.x names;
- users migrate with a provided codemod and migration guide;
- users who need the old API pin `dartpy<7`, which the 6.20.x maintenance line
  serves.

Earlier runtime-shim attempts produced untyped stubs, alias-order bugs and broken
introspection. Native renames in the binding sources avoid all three.

## Package and naming

- The distribution and import name stays `dartpy`; documentation uses
  `import dartpy as dart`.
- `dartpy` is a Python package: a private native extension (`dartpy._core`)
  plus pure-Python helper modules that return native objects. There are no
  Python wrapper classes, with one exception: the small native capability
  handles described under "Model vocabulary".
- Every public type has exactly one canonical module, set explicitly through
  `__module__`. CI rejects duplicates and any `dartpy._*` module that leaks into
  a public `__module__`.
- The top level re-exports exactly this allowlist: `World`, `WorldBatch`,
  `Multibody`, `Link`, `Joint`, `Dof`, `Frame`, `SimpleFrame`, `Collider`,
  `Visual`, `Transform`, `Rotation`, `Material`, `PositionServo` and
  `VelocityServo`, plus `Recorder` when the recorder (M4) ships. Everything else
  lives in its submodule.
- Python names follow PEP 8:
  - `snake_case` for functions, methods, keyword arguments and fields;
  - `CapWords` for classes;
  - `UPPER_CASE` for enum members (no `export_values`).
- Properties expose cheap state; array-valued properties return read-only
  copies. Methods expose computation and mutation, and accept `out=` where they
  fill arrays.
- Options beyond one or two obvious positional arguments are keyword-only.
- Errors raise exceptions; bad input is never logged and ignored:
  - all DART errors share one base class, named in M5's errors PR;
  - validation errors subclass `ValueError`;
  - lookup errors subclass `KeyError` and list the valid names;
  - loaders raise instead of returning `None`.
- DART's C++ diagnostics are forwarded to `logging.getLogger("dartpy")`.
- Typed stubs (`.pyi` plus `py.typed`) ship in every wheel. CI fails on stub
  drift.

A single reviewed rename table drives the binding-source rename, the stubs, the
user codemod and the migration guide. Docstrings generated from the C++
documentation pass through the same table.

## Model vocabulary

Python 7.0 applies three substitutions to every identifier:

| C++ (unchanged in 7.0) | Python 7.0 |
| --- | --- |
| `Skeleton` | `Multibody` |
| `BodyNode` | `Link` |
| `DegreeOfFreedom` | `Dof` |

Examples: `MetaSkeleton` becomes `MetaMultibody`, `SoftBodyNode` becomes
`SoftLink`, and `getNumBodyNodes` becomes `num_links`. `World`, the `Joint`
family, `Frame` and `SimpleFrame` keep their names. `Frame` stays the base
class; a public name is never reused with a new meaning.

`ShapeNode` and `ShapeFrame` are not public in Python. They are replaced by two
native capability handles on the same node:

- **`Collider`**: shape plus contact and dynamics properties. Collision contexts
  (contacts, collision objects, ray casts) return `Collider`s, for example
  `contact.collider_a`.
- **`Visual`**: shape plus appearance. Render contexts return `Visual`s.
  `SimpleFrame` visuals also get a `Visual` handle.

Both handles compare and hash by node identity, raise a defined error once
detached, and share geometry and local pose when one node carries both aspects.

Collections such as `world.multibodies`, `multibody.links`, `multibody.joints`
and `multibody.dofs` accept an index or a name. They support `len`, iteration
and `.names`, and raise `KeyError` listing the valid names.

The C++ API adopts the same vocabulary after the 7.0 release, through a
mechanical rename:

- the old class names stay as type aliases, including in the
  forward-declaration headers;
- the old methods stay as forwarders covering every overload, const variant and
  default argument;
- virtual member functions and nested type names keep their names, because
  downstream code overrides and names them;
- new names never clash with downstream `Gz*` types.

## Reserved names

Use these spellings consistently:

- `compile()` and "compiled" describe simulation mode. Never "bake" in public
  text.
- The simulation counter is `world.step_count`, never "frame". The world frame
  is `dart.Frame.world()`.
- `every=` is the only decimation keyword.
- Angles are in radians everywhere. No API takes degrees implicitly.
- `ActuatorType.VELOCITY_MOTOR` names the legacy ideal velocity motor (the C++
  `SERVO`), to distinguish it from the new `VelocityServo` actuator.
- Sensor classes are `IMUSensor`, `ForceTorqueSensor`, `ContactSensor` and
  `JointStateSensor`. An IMU reports orientation as a `Rotation`.
- `dart.Recorder` is the Python recorder; the C++ `Recording` class is not
  exposed under that name.
- Builder specifications (`*Spec`, immutable and keyword-only) are distinct from
  the bound C++ `*Properties` structures.

## World, modes and mutation classes

A `World` owns model membership and advances time. Its public Python surface is
capped at about 20 names, enforced by `scripts/nanobind/api_surface.py`, and
every new member needs a use case:

- constructor: `World(*, name, time_step, gravity, collision_detector, threads)`;
- properties: `name`, `time_step`, `gravity`, `time`, `step_count`,
  `multibodies`, `frames`, `is_compiled`;
- methods: `add`, `remove`, `add_rigid_body`, `edit`, `compile`, `step`,
  `reset`, `clone`, `capture_state`, `restore_state`, `capture_checkpoint`;
- sub-objects: `world.collision`, `world.solver`, `world.sensors`,
  `world.hooks` and `world.constraints`.

The separate keyword-only overload `World(*, config=...)` consumes a typed
`WorldConfig`; mixing it with individual constructor options raises `TypeError`.
Python configuration defaults select DART, while the C++ defaults remain FCL.
The bound configuration fields exclude allocator internals.

dartpy's `dart.World` and `dart.io.load_world` for MJCF default to the native
DART collision detector, exposed through `world.collision.detector`. The
MJCF switch belongs only in `dart.io.load_world`; the C++
`dart::utils::MjcfParser::readWorld` and its direct dartpy `MjcfParser`
binding keep FCL. Changing that parser would also switch C++ callers. The
C++ `World` defaults to FCL, for Gazebo compatibility.

Not bound in Python: internal preparation steps, the C++ recording accessor
(replaced by `dart.Recorder` once the recorder ships), index and memory-manager accessors, the
`remove_all_*` family, and boolean collision-check overloads.

Worlds have two explicit modes on the same objects:

- **Editing mode.** The object graph is freely mutable.
- **Simulation mode.** `world.compile()` prepares dense internal data, may
  restructure internal storage for locality while public order stays fixed,
  and freezes topology. Steady-state stepping performs **zero dynamic
  allocation** on the DART and Bullet collision detectors.

Worlds that were never compiled stay lazy, which is the C++ default. They
prepare the same dense data on their first step and rebuild only the affected
parts after an edit, keeping object addresses stable, so downstream runtime
edits keep working. Lazy and compiled worlds run the same kernels.

Structural edits on a compiled World happen inside `with world.edit():`. Leaving
the block recompiles the affected data and starts a new topology revision.
Outside `edit()`, a structural edit on a compiled World raises
`ModelFrozenError`. C++ strictness is opt-in.

Every mutator belongs to exactly one class:

| Class | Allowed while compiled | Examples | Effect |
| --- | --- | --- | --- |
| Topology | Only inside `edit()` | add/remove multibody, link, joint, node or aspect; joint type changes; re-parenting, splitting or merging; shape replacement; collision-group membership; detector or solver replacement; manual constraints; attaching or detaching actuator rows; sensors and hooks | New topology revision |
| Parameter | Yes | mass, inertia, center of mass; limits; spring, damping, rest position, friction; actuator gains and bounds; contact material; gravity; time step; actuator kind on lazy worlds | New parameter revision, no allocation |
| State and control | Yes | positions, velocities, commands, external forces, actuator targets | No model revision; caches invalidate and sleeping bodies wake as today |
| Cosmetic | Yes | appearance, material, visibility, shadows, segmentation id | None |

Revisions are tracked per World, not process-wide. Parameter setters stop
bumping the process-wide structural version, and changing a joint's actuator
kind bumps a revision, because articulated-body dynamics branch on it.

Strict enforcement has two tiers:

1. DART-owned mutators check the owning World and raise before changing state.
   Adding a Multibody that another World owns raises.
2. Writes through raw references (such as collision-option fields or
   collision-group contents) are detected at the next `step()` or `compile()`,
   which raises. This tier is documented as detected late.

Python exposes typed setters instead of raw references. Tests cover every
mutation path downstream simulators use: re-parenting, adding and removing
models, shape poses, inertia, and detector and solver replacement.

## State, arrays and persistence

- Array-returning properties return **read-only copies**, on both the NumPy
  fast-export path and its fallback. In-place writes raise instead of silently
  doing nothing, and the meaning never changes.
- `out=` arguments fill caller-provided buffers without allocating. They copy;
  they never alias world storage.
- Zero-copy live views, when available, are separately named, with documented
  strides, lifetime and stale-view errors.
- `world.capture_state(out=None)` returns a **dynamic snapshot**: positions,
  velocities, control channels and time. Snapshots carry a structural digest of
  the model (multibodies, joints, degrees of freedom and stable identifiers, in
  order). They restore across parameter revisions into the originating World,
  its remapping clones, or any World whose structural digest matches.
  Revision numbers are per World and are never compared across Worlds. A
  mismatched digest raises. `WorldBatch.restore_state(state, indices=...)`
  broadcasts one snapshot to selected clones.
- `world.capture_checkpoint()` returns an **exact checkpoint**. It covers
  declared built-in configurations at completed-step boundaries on the same
  build and platform. Restoring it re-steps bit-identically. A mismatched
  topology, parameter set, build, solver or extension raises
  `CheckpointMismatchError`.
- Restoring never goes through ordinary setters, which have side effects (waking
  sleeping bodies, bumping revisions). It restores the following directly:
  - sleep state and the World's sleep records;
  - adaptive soft-body activation;
  - contact manifolds and warm starts;
  - solver history and constraint impulses;
  - random-number-generator state;
  - the time step and solver settings;
  - control channels.
- Each collision detector declares which history it keeps and how it is
  rebuilt. Extensions that declare nothing make checkpoint capture fail. Python
  step hooks must be stateless or implement `__getstate__`.
- The checkpoint oracle compares the serialized fields and every re-stepped
  frame (contacts, impulses, accelerations, forces, sleep and activation), not
  raw memory.
- Saved snapshots, checkpoints and recordings, including their pickles, carry a
  magic number and a version. Golden files are written by 7.0.0, and a 7.x
  release reads every format written by an earlier 7.y release.

## Control and extension

Legacy C++ `World::step` semantics are unchanged: `step(true)`, the default,
clears commands and internal and external forces after each step, and
`step(false)` keeps them.

The Python API adds layered extension points, from most to least common:

1. **Native actuator channels.** These are separate target storage that persists
   until changed and is applied before each substep. `world.step(n)` equals `n`
   single steps.
2. **User-owned loops** with `world.step(n)` decimation and batched arrays.
   `step` releases the GIL and rejects `bool` arguments with a message that
   points to target channels. The codemod flags every non-literal `step(x)`
   call.
3. **`StepHook`** with `every=k`, called at deterministic boundaries.
4. **Compiled hooks**, available once contiguous state exists. They use a
   narrow, versioned plain-C ABI.
5. **C++ hooks** in native code.

### Compliant motor rows

`PositionServo` and `VelocityServo` are opt-in, bounded compliant motor rows
solved together with contacts and limits. With time step `h`, stiffness `K`,
damping `D`, position target `r`, velocity target `v_d`, position `q`,
post-step velocity `v⁺` and effort bounds `τ_min`, `τ_max`, each row solves for
the impulse

```text
p = clip(b - c·v⁺, h·τ_min, h·τ_max)
b = h·K·(r - q) + h·D·v_d
c = h·D + h²·K
```

- This is an implicit PD controller with effort saturation inside the solve, so
  reactions under contact are correct. The zero-gain case is defined
  explicitly.
- Rows come after position limits in constraint priority. Scalar joints come
  first.
- The reported effort is the impulse divided by `h`, kept separate from the
  joint force fields.
- Joints without an actuator stay bit-identical. Passive springs and command
  resets are unchanged. The constraint-size cost is measured before the rows
  ship.

### Determinism and the executor

- Bit-identical results hold within a pinned compiler, flags, Eigen
  configuration, instruction set and floating-point environment.
- One deterministic executor changes scheduling only. It preserves parallel
  eligibility, phase barriers and indexed outputs, and folds results in serial
  order, with no per-thread partial sums.
- Collision filters run on the submitting thread. Python callbacks run only at
  deterministic executor boundaries.
- Randomized solver passes stay serial. Moving their random-number state into
  each solver is a labeled numeric change with a multi-World rebaseline.
- Tests straddle the parallel thresholds: 127, 128 and 129 islands, and 255,
  256 and 257 pairs.
- Dense kernels preserve traversal and child-summation order, Eigen expression
  grouping, the fused multiply-add contraction policy, factorization choices
  and lazy-cache timing. They keep a virtual fallback for custom subclasses.

### Concurrency

- A World has one stepping owner. Re-entrant stepping and concurrent mutation
  raise.
- While a World steps, any access from another thread raises
  `WorldBusyError`. Reads count, because getters fill lazy caches. The busy flag
  is set before the GIL is released.
- The passive viewer reads only renderer-owned buffers that the stepping thread
  fills during `viewer.sync()`.

### Native extensions

The wheels link DART statically, so a second native copy would duplicate its
global state. Third-party native extensions that operate on dartpy-owned
objects are therefore not supported with the wheels; they need a shared DART
installation. Python is the extension point, and later the plain-C hook ABI.
The wheel extension should export only `PyInit__core`. It does not yet: it also
exports symbols from the libraries it links statically, and no wheel check
covers its export list.

## Batching

`dart.WorldBatch` steps independent compiled clones in parallel, on one
deterministic executor shared across worlds.

Clones are produced by a remapping clone that copies the following by stable
identifiers:

- collision filters, including those the MJCF loader installs;
- contact-surface handlers, each through a clone hook;
- manual constraints;
- actuator channels;
- sensors, re-bound to the clone;
- hooks;
- time and configuration.

Anything that cannot be cloned, such as a handler without a clone hook, makes
construction raise. Nothing is silently dropped. Clone `i` derives its random
seed from the base seed and `i`.

The gate for remapping clones uses URDF, SDF, MJCF-with-weld and custom-filter
scenes. Without randomness, clone `i` stepped `n` times matches the source
stepped `n` times. With randomized solver passes or seeded sensor noise, clone
`i` matches a serial reference World seeded like clone `i`.

Batch arrays use the World's degree-of-freedom order, shaped `(count, ...)`.

- Writes go through `batch.set(field, values, *, indices=None, mask=None)`.
- Per-clone parameters go through `batch.set_parameter(...)`, which starts a new
  parameter revision without recompiling.
- Broadcasting accepts a scalar against a batch, or equal shapes.

## Transforms, rotations and conventions

`dart.Transform` and `dart.Rotation` are immutable owning value types, scalar or
batched, backed by the same rigid-transform storage the C++ API uses, with a
scalar fast path.

- `@` composes transforms and applies them to `(..., 3)` points. `*` is not
  defined.
- Constructors are keyword-only and accept exactly one rotation form, such as
  `rotation=` or `wxyz=`/`xyzw=`. Roll-pitch-yaw construction is extrinsic
  X-Y-Z, i.e. `Rz(yaw) Ry(pitch) Rx(roll)`, matching URDF and SDF. The complete
  constructor set is settled in M5's transform PR.
- Inputs are validated (finite values, `‖RᵀR − I‖_F`, `|det R − 1|`, the
  homogeneous row) and raise `ValueError`. The default tolerance is calibrated
  in M5, and the error suggests `normalize=True`. That option projects onto the
  nearest rotation; reflections and shear still raise. Composition never
  normalizes silently, and drift is documented.
- **Quaternion order is always named.** Inputs use `wxyz=`/`xyzw=`; outputs use
  `.wxyz`/`.xyzw`. No API accepts or returns an unnamed 4-vector. `wxyz` is the
  canonical display order.
  - Normalization uses a scaled norm.
  - Sign canonicalization (`w >= 0`, with a deterministic tie rule) is a
    separate, documented policy.
  - Matrix-to-quaternion conversion uses the numerically stable branch on the
    largest diagonal term.
- Pickling keeps the exact matrix bits.
- Frame queries: `link.world_transform` is `X_WL`, `link.relative_transform` is
  the pose in the parent frame, and `link.get_transform(relative_to=frame)` is
  `X_FL`. `X_AB` always means the pose of frame `B` expressed in frame `A`.
- Spatial vectors are ordered `[angular; linear]`. Named 3-vector accessors
  (`linear_velocity`, `angular_velocity`) are preferred.
- Cameras use the optical frame convention: `+Z` forward, `+X` right, `+Y` down,
  with `K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`. `Camera.look_at(...)` avoids
  manual axis handling.

Binding rules:

- `__matmul__` and `__rmatmul__` are written by hand;
- no implicit conversions, which would turn validation errors into overload
  failures;
- `__array_ufunc__ = None`, so NumPy defers to `@`;
- roll-pitch-yaw construction does not reuse the existing XYZ Euler helper,
  whose convention differs;
- every C++ function that takes a non-const rigid-transform reference is
  audited before binding.

## Loading and building models

- `dart.io.load_model(path, *, model=None, fixed_base=..., package_dirs=...)`
  returns one detached `Multibody`. It rejects files that describe several
  models, unless `model=` selects one.
- `dart.io.load_world(path)` returns a `World` with its settings. MJCF files are
  world-only, because their collision rules and equality constraints are owned
  by the world.
- `world.add()` accepts only a detached `Multibody` or `SimpleFrame`, and raises
  `NameConflictError` instead of renaming silently.
- Programmatic construction uses `add_*` builders with immutable keyword-only
  specifications on the live objects, for example
  `multibody.add_link(name, parent=..., joint=dart.dynamics.RevoluteJointSpec(...))`,
  `link.add_collider(...)` and `link.add_visual(...)`.
- `world.add_rigid_body(...)` returns the `Link` of a new one-link `Multibody`.
- Constraints are added through `world.constraints.add(...)` inside `edit()`.

## Rendering

M6a merged the Filament renderer on `main` in
[#3696](https://github.com/dartsim/dart/pull/3696). Its camelCase `dartpy.gui`
bindings are interim and are not part of the 7.0 API. M6b implements the
following Python API and deletes that module; the
[project home](../dev_tasks/dart7_python_first/README.md#release-train-and-checkpoints)
owns its schedule and release gates:

- **`dart.render`** is headless.
  - A `Renderer` returns NumPy arrays: RGB, metric depth and segmentation.
  - Cameras are described separately and attached to frames.
  - An `Environment` controls lighting.
  - It never initializes windowing.
  - `renderer.lookup_segmentation(id)` returns the `Visual`, `Link` or
    `Multibody` behind a segmentation id.
- **`dart.viewer`** offers `open(world, ...)`, a passive viewer with a user-owned
  loop, and `launch(world, ...)`, a managed loop.
  - The managed loop is a native workspace drawn by private C++ in the
    extension (D31b): scene navigator, scene panels, inspector, tools,
    profiler and log console.
  - Python passes scenes (a World factory, key actions and light step hooks)
    and declarative panel controls, and never calls ImGui.
- **`dart.Material`** is a core PBR appearance value: base color (RGBA),
  metallic, roughness, emissive and textures. It is stored with each visual's
  properties, so it is available without a renderer, and editing it is a
  cosmetic change. Adding it to the visual properties is an accepted ABI change.
  Precedence:
  1. an explicit `Visual.material`;
  2. the asset's own material, whose mesh material gains PBR fields filled from
     glTF;
  3. the legacy RGBA color.

  Color and alpha modes stay internal. The segmentation id is a field of the
  visual properties.

Importing `dartpy` and stepping a World must work on hosts without any graphics
libraries.

## Packaging and versioning

- Wheels target Python 3.12 and newer, as one stable-ABI wheel per platform:
  Linux x86_64 and aarch64, macOS arm64, Windows x64. There is no source
  distribution. This replaces the Python 3.10+ support and `cp310`/`cp311`
  wheels that main's unreleased changelog entries describe; M5's packaging
  PR updates those entries.
- The C++ package accepts requests for any earlier version back to 6.0, until
  a release removes APIs downstreams use. A custom version file provides this:
  CMake's stock `AnyNewerVersion` would also accept requests for older majors,
  which DART does not promise to support.
  - Downstreams opt out with a version range such as
    `find_package(DART 6.10...<7)`.
  - When a 6.x request resolves to 7.x, the package configuration prints a
    status line.
  - A CMake script test renders the version file for several releases and
    checks which requests it accepts.
  - The version macros downstreams use, such as `DART_VERSION_AT_LEAST`, stay.
- SemVer covers the Python API and the documented C++ headers at source level.
  The C++ ABI, and with it the SONAME (MAJOR.MINOR), changes with every minor
  release.
- Release candidates use a Python version suffix; the C++ version stays
  numeric.
