# Lie Group Batch Processing

## Status

Accepted — initial thin layer landed with the typed Lie group API
(`dart/math/lie_group/`).

## Context

DART's experimental, ECS-based simulation engine (`dart/simulation/`)
performs Lie group math (composition, exp/log, adjoint) per entity. As entity
counts grow, doing this one element at a time leaves throughput on the table:
the integration loop (`compute/world_step_stage.cpp`,
`RigidBodyIntegrationStage`) already advertises `Simd | DataParallel` intent but
currently iterates scalar per-entity math.

The question: should batch support be a **separate** API/type set (e.g.,
`SO3Batch` holding N rotations in struct-of-arrays form) or **consolidated** into
the existing single-element Lie group types?

## Decision

**Consolidate.** Do not introduce a parallel batched type hierarchy. Instead,
expose batch operations as thin free functions that view packed, contiguous
buffers element-wise through the existing `Eigen::Map<SO3<S>>` /
`Eigen::Map<SE3<S>>` specializations and apply the existing scalar operation.

See `dart/math/lie_group/batch.hpp`: `composeBatch`, `expBatch`, `logBatch`,
and `adjointBatch`.

Buffer layout is AoS of `Params` (each element occupies `T::ParamSize` scalars;
tangents occupy `T::DoF` scalars), which is exactly what the `Map`
specializations consume — so the batch path is zero-copy over caller-owned
buffers.

## Rationale

- **No second type system.** A separate SoA type would duplicate every operation
  (`exp`, `log`, `Ad`, `operator*`, …) and drift from the single-element types.
  The `Map<SO3<S>>` / `Map<SE3<S>>` specializations already provide typed,
  zero-copy element access into external storage; the only missing piece was the
  iteration/kernel glue.
- **Matches the existing precedent.** `dart/math/geometry.hpp` already ships
  `AdT_batch` / `AdInvT_batch` / `dAdT_batch` / `dAdInvT_batch` as free functions
  over pointer arrays, transposing AoS→SoA internally and using `dart/simd`
  (`simd::transposeAosToSoa`, `simd::cross3`) with a scalar tail. The new batch
  layer follows the same shape for the typed API.
- **Matches project guidance.** `docs/design/scalable_compute_decisions.md`
  ("SIMD Constraints") and `docs/onboarding/api-boundaries.md` require SIMD/batch
  details to stay _behind implementation boundaries_, with scalar fallbacks and
  preserved Eigen interop — i.e., not exposed as an architecture-specific public
  type.
- **Fits the consumer's actual layout.** The experimental world stores transforms
  AoS in entt component pools; there is no SoA transform store today that would
  justify a SoA type.

## Implementation notes

- The functions in `batch.hpp` are the always-correct **scalar reference path**.
  SIMD acceleration (via `dart/simd`), multi-core scheduling, and opt-in CUDA
  kernels are expected to be slotted in _behind the same interface_ — callers do
  not change and public headers do not expose backend-specific types.
- The experimental engine is intentionally **not** rewired in this initial
  layer; that is a follow-up once a kernel is benchmarked against the scalar
  path.

## When to revisit

Escalate to a dedicated SoA storage type (separate arrays per component, e.g.
quaternion `x/y/z/w` planes) only if benchmarks on the experimental engine prove
the interleaved `Map`-over-`Params` layout is the bottleneck. Until then,
consolidated wins on maintainability.
