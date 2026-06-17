# Batched World And Device Residency

This note is the PLAN-091 WP-091.33 contract for replicated batched worlds and
future device residency. It narrows the batching target to semantics first:
batched execution must be observationally equivalent to stepping `n`
independent sequential `World` instances, while leaving room for CPU, SIMD, and
device implementations behind the same internal Model/State split.

## Decision

DART should support two batch execution shapes:

1. **Canonical homogeneous batch.** A baked immutable Model is shared across
   batch lanes, mutable State/Control/Contacts blocks carry a leading
   `worldCount` dimension, and each lane evolves independently. This is the
   direction used for replicated environments, rollout benchmarks, and future
   resident device execution.
2. **Heterogeneous fallback.** Existing `World` objects remain independent and
   are scheduled as separate executor nodes. This preserves exact semantics for
   mixed topologies and dynamic scene-authoring workflows, but it is not the
   target layout for accelerator throughput.

The public API should describe execution shape, not backend technology. Device
buffers, streams, queues, kernels, residency handles, and component-storage
types stay internal. Public `World` handles remain the user-facing authoring
surface; batched SoA buffers are the execution representation.

## Semantics Contract

For a homogeneous batch with `worldCount = n`, lane `i` is equivalent to a
separate `World` stepped from the same baked Model plus lane-specific State,
Control, Contacts, random seeds, and time cursor.

Required equivalence:

- A single-lane batch is bitwise identical to the ordinary sequential
  `World::step()` reference path for the supported solver families.
- Multi-lane batch results match stepping each lane independently with the same
  substep count, solver options, control inputs, and contact ordering.
- Errors and diagnostics include the lane index when a lane-local validation or
  solve failure occurs.
- No lane may read or mutate another lane's State, Control, Contacts, scratch,
  random stream, diagnostics, replay state, or allocator scratch.
- Structural edits are outside a resident batched step. If a user changes
  topology, the batch must leave resident execution, rebuild the baked Model,
  and invalidate topology-dependent device buffers.

The batch contract intentionally does not promise lockstep collision broad-phase
work sharing, cross-lane coupling, asynchronous completion, or identical
floating-point results across different backend precision policies.

## State Blocks

The canonical layout uses immutable Model blocks and mutable State blocks:

- **Model:** topology, dense indices, masses, inertias, rest shapes, material
  parameters, solver-family configuration, and stage schedule metadata. Model
  blocks are built at bake/finalize time and are read-only during a rollout.
- **State:** positions, orientations, velocities, generalized coordinates,
  deformable nodes, warm starts, contact caches that are semantically part of a
  lane, current time, and frame counter. State carries the leading world
  dimension.
- **Control:** forces, torques, prescribed targets, actuator commands, policy
  inputs, and external callbacks that have been lowered to data. Python or C++
  callbacks do not run inside backend compute nodes.
- **Contacts:** broad-phase candidates, narrow-phase contact manifolds,
  constraint rows, friction rows, and contact diagnostics for the current
  substep. Contacts are rebuilt per lane unless a future backend proves a
  shared immutable candidate structure is equivalent.
- **Scratch:** per-step temporary buffers. Scratch is neither public State nor
  persistent Model; each backend owns its scratch lifetime.

Existing `RigidBodyStateBatch`, `RigidBodyModelBatch`, and
`RigidBodyControlSequenceBatch` are the current canonical-direction seed:
host-owned rigid-body SoA with a leading world dimension and step-major Control
sequence storage. `BakedRigidBodyBatchOwner` is the current host owner for the
Model/State blocks: it captures mutable State repeatedly while refreshing
immutable Model storage only when the baked dense-index identity changes.
Existing `stepWorldsBatched()` is the heterogeneous-fallback seed: it schedules
independent Worlds through the executor without introducing a shared SoA
execution representation. Rollout diagnostics record whether a call resolved to
the homogeneous SoA path or this heterogeneous fallback.

## Device Residency

A future resident backend should treat the device State buffer as the source of
truth while a batch is resident. Host `World` objects and handles are
authoring/synchronization views at explicit boundaries, not per-step mirrors.

Residency rules:

- Residency is an internal owner attached to a baked batch, not a public device
  type.
- Model upload happens after bake and is invalidated by topology or solver
  configuration changes.
- State upload/download is explicit at batch boundaries. A step that runs on a
  resident backend must not copy State to the host unless the caller asks for a
  host snapshot, diagnostics, or replay output.
- Control upload is per step or per rollout segment, depending on the control
  sequence shape.
- Contact and scratch buffers are backend-owned. They may stay resident across
  substeps, but their contents are invalid outside their documented validity
  window.
- Backend failure falls back only at a synchronization boundary where State can
  be made coherent. Mid-step CPU fallback is allowed only if the backend proves
  the partially updated State is either untouched or recoverable.

This follows the packaging rule in `scalable_compute_decisions.md`: the default
core and default `dartpy` wheel stay CPU-only, while optional sidecars may own
device runtime code.

## Precision Policy

Double precision is the reference. A backend may offer float or mixed precision
only when it records the resolved precision in the benchmark/diagnostic packet
and passes a declared tolerance gate against the double reference.

Policy:

- CPU and scalar/SIMD reference paths use double unless a solver family already
  has a documented scalar-type policy.
- Device backends may use double, float, or mixed precision per stage, but the
  precision is resolved at bake and reported in diagnostics.
- A public API must not expose a backend-specific scalar type. If user-facing
  precision control is needed, expose a backend-neutral policy value.
- Claims about speedups must report whether transfer time is included and must
  compare against the double reference or explain the accepted tolerance.

## Current Seeds

- `dart/simulation/compute/rigid_body_state_batch.hpp`:
  canonical-direction seed for homogeneous SoA State/Model/Control blocks,
  rollout execution-shape diagnostics, and the `BakedRigidBodyBatchOwner` host
  owner.
- `dart/simulation/compute/world_batch.hpp`: heterogeneous-fallback seed for
  independent per-World executor scheduling.
- `dart/simulation/compute/world_step_stage.hpp`
  `BatchedRigidBodyIntegrationStage`: live-World extraction/apply seed for a
  single rigid-body stage.
- `dart/simulation/compute/cuda/rigid_body_state_batch_cuda.*`: optional
  sidecar seed for resident batch rollout kernels.

## Follow-Up Packets

The WP-091.33 design slices into these implementation packets:

- **WP-091.33a Batch semantics tests:** add one-lane and multi-lane parity
  tests for the supported rigid-body batch paths, including lane-indexed error
  text for validation failures.
- **WP-091.33b Baked rigid Model/State owner:** promote the rigid batch seed
  from extraction/apply helpers into an internal baked owner that can be reused
  across rollout segments without rebuilding immutable Model blocks.
- **WP-091.33c Control-sequence rollout shape:** add a backend-neutral Control
  sequence layout and rollout API that keeps Python callbacks outside compute
  nodes and records the resolved execution shape.
- **WP-091.33d Resident device owner:** introduce an internal residency owner
  for the optional sidecar path with explicit upload/download/sync boundaries
  and a CPU fallback only at coherent boundaries.
- **WP-091.33e Precision and packet reporting:** extend benchmark and
  diagnostics packets to record backend, precision, transfer inclusion, and
  lane count for batched executions.

Each packet must preserve `World::step()` sequential semantics and keep backend
device details out of public C++ and Python APIs.
