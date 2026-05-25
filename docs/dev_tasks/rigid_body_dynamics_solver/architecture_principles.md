# Rigid-Body Solver — Architecture Principles (DART 7 experimental → DART 8)

Durable design intent for the experimental rigid-body dynamics solver. These
govern every slice (integration, contacts/constraints, model loading) and how
the PR #2705 convention re-alignment should be resolved
([`convention_realignment.md`](convention_realignment.md)). Roadmap context in
[`RESUME.md`](RESUME.md).

## Principles

1. **DART-6 dynamics parity.** Same/similar algorithms and numerical results as
   DART 6's rigid-body dynamics (Featherstone articulated dynamics, contact /
   constraint LCP, semi-implicit integration). Bodies fall under gravity,
   contacts resolve, energy/momentum behave as in DART 6. Validate against DART
   6 behavior, not just internal self-consistency.

2. **ECS-native, consolidated (SoA) data.** Lay out per-body / per-DOF state as
   Structure-of-Arrays contiguous buffers (the #2698
   `RigidBodyStateBatch` / `RigidBodyModelBatch` pattern), not per-entity AoS
   objects, so kernels stream over contiguous memory and many worlds/bodies pack
   together.

3. **Cache-friendly + batch processing.** Hot loops operate over batches /
   contiguous arrays; avoid pointer-chasing, per-entity heap objects, and
   virtual dispatch in inner loops. Prefer data-parallel formulations.

4. **Backend-portable, hybrid compute.** Hot kernels are **pure functions** of
   `(state, params, forces)` with no hidden world/global coupling, expressed
   through the `ComputeExecutor` / `ComputeGraph` boundary so the same kernel
   runs on multi-thread CPU, SIMD, and CUDA — and hybrid combinations. Keep
   `integrateRigidBodyStateBatch*` and any new kernel free of side effects and
   `World`/registry lookups; gather inputs upstream, scatter outputs downstream.

5. **Share DART 6 code where aligned; rewrite when not.** Reuse DART 6 math and
   algorithms (Lie groups / spatial algebra, LCP solvers, dynamics formulas)
   wherever they fit the SoA / batch / pure-kernel API. Where DART 6's structure
   (AoS, `BodyNode` trees, virtual dispatch) does not fit, **rewrite from
   scratch** — DART 8 ships only the experimental API, so there is no obligation
   to preserve DART 6's internal structure for compatibility.

## Implication for gravity / forces (and the #2705 re-alignment)

Reconcile in the **batch-friendly direction**, not by reverting to per-entity
gravity:

- Keep the integration kernel **pure** (no gravity baked in) — this is #2698's
  design and is required for backend portability (principle 4).
- Preserve DART-6 "bodies fall" behavior (principle 1) via a **batch-friendly
  gravity / force-assembly stage** that fills the SoA force buffer with
  `mass * gravity` (+ external/applied forces) before the integrator runs, on
  every backend. The integrator then consumes that buffer.
- Force/torque components follow #2698's persistent-input convention: a step
  reads them into a transient SoA force buffer and leaves the components intact.
  Gravity is added only to that transient buffer as `mass * gravity`.

This keeps the pure/batched/portable architecture **and** DART-6 behavior,
rather than trading one for the other.
