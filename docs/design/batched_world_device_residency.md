# Batched World And Device Residency

This note owns the replicated-world/device-residency contract originating in
completed PLAN-091. Current qualification is owned by PLAN-030/080/041/122
under the [PLAN-040 milestones](../plans/040-dart7-release-hardening.md). It narrows the batching target to semantics first:
batched execution must be observationally equivalent to stepping `n`
independent sequential `World` instances, while leaving room for CPU, SIMD, and
device implementations behind the same internal Model/State split.

## Decision

DART should support two batch execution shapes:

1. **Canonical homogeneous batch.** A baked immutable Model is shared across
   batch lanes, mutable State/Control/Contacts blocks carry a leading
   `worldCount` dimension, and each lane evolves independently. This is the
   direction used for replicated environments, rollout benchmarks, and resident
   device execution.
2. **Heterogeneous fallback.** Existing `World` objects remain independent and
   are scheduled as separate executor nodes. This preserves exact semantics for
   mixed topologies and dynamic scene-authoring workflows, but it is not the
   target layout for accelerator throughput.

The public API describes execution shape and may provide a small DART-owned
device preference/report; runtime technology stays behind that value boundary. Device
buffers, streams, queues, kernels, residency handles, and component-storage
types stay internal. Public `World` handles remain the user-facing authoring
surface; batched SoA buffers are the execution representation.

## Semantics Contract

For a homogeneous batch with `worldCount = n`, lane `i` is equivalent to a
separate `World` stepped from the same baked Model plus lane-specific State,
Control, Contacts, random seeds, and time cursor.

Required equivalence:

- A single-lane batch is bitwise identical to its ordinary sequential
  reference only for a pinned deterministic backend, variant, precision and
  update order. Cross-backend comparisons use declared finite-horizon norms
  and tolerances even when both paths use Float64.
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

Resident backends should treat the device State buffer as the source of truth
while a batch is resident. Host `World` objects and handles are
authoring/synchronization views at explicit boundaries, not per-step mirrors.
The current optional CUDA sidecar implements the first rigid-batch resident
owner behind this contract; future backends should follow the same ownership
rules.

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
- Explicit device requests never silently fall back. An automatic policy may
  choose only a documented coherent boundary and supported path; failed partial
  updates require rollback or an explicit invalid-state result. Drain device
  work before releasing captured buffers or reconstructing an invalid context.

This follows the packaging rule in `scalable_compute_decisions.md`: the default
core and default `dartpy` wheel stay CPU-only, while optional sidecars may own
device runtime code.

Each in-flight State owns mutable graph execution records, bindings, scratch
and continuation. Shared pools have one total worker budget. Invalidate caches
on topology/configuration/buffer/capacity changes and restore; active contact
counts within prepared capacity are data. Device launch is not completion:
use events or synchronous completion before dependent work and public return.
Serialize semantic state/versioned configuration, then reconstruct runtime
resources; see [compute decisions](scalable_compute_decisions.md).

## Precision Policy

M1 requires Float64 on both CPU and CUDA for every accepted example. The
policies below describe later precision choices; public `dtype`/scalar-template
selectors remain deferred under PLAN-041.

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
- Batched benchmark packets must record the resolved backend, resolved
  precision, whether transfer time is included, lane count, step count, and the
  resolved execution shape for every representative batched row. The Phase 5
  CUDA packet checker enforces these fields for new batched evidence rows so a
  timing result cannot be separated from its precision or transfer policy.

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
  sidecar seed for resident batch rollout kernels and the internal
  rigid-batch resident owner.

## Historical Packet Decomposition

The completed WP-091.33 design used the following decomposition. These IDs
are historical context, not executable active packets; consult the current
PLAN-030/080/041/122 packets and source assessment before choosing work:

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
