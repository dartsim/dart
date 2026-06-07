# Shared Experimental CUDA Device Substrate

## Status

Implemented baseline. PR #2875 landed the shared CUDA runtime helpers,
`DeviceBuffer`, and single-sourced rigid orientation core on `main`. This
document now owns the durable rationale and extraction triggers for sharing GPU
device-runtime code across DART 7's experimental CUDA solvers instead of
reinventing it per solver. Operating state (priority, horizon, next step, gate)
lives in `docs/plans/dashboard.md` under PLAN-031. This design is scoped to
`dart::simulation::experimental`; it does not change the classic
`dart::simulation::World`, any public API, or the GPU packaging shape decided in
[`scalable_compute_decisions.md`](scalable_compute_decisions.md).

## Purpose

DART 7 gained a private, opt-in CUDA backend, and the number of CUDA-bearing
solvers is growing: the Phase-5 homogeneous-batch rigid-body integrator
(PLAN-030), the Vertex Block Descent solver (PLAN-104), and the deformable
projected-Newton PSD projection (PLAN-081) all ship CUDA today, with AVBD-CUDA
(PLAN-104), rigid-IPC GPU (PLAN-082), PD-IPC GPU (PLAN-081), and broadened
Phase-6 GPU stages (PLAN-030) queued behind them.

Each CUDA module currently re-implements the same device plumbing — runtime
probing, error mapping, launch configuration, device-buffer lifetime. Left
unchecked, every new GPU solver pays that cost again, the variants drift, and
test coverage fragments. This document decides what device code should be shared
**now**, what should stay solver-local until a second consumer proves it shared,
and which dedicated libraries (math, collision, numerical optimization, data
structure) are justified versus speculative.

The governing rule is DART's simplicity axiom (`docs/ai/principles.md` #3): a
shared block is extracted only when **two or more current consumers already
duplicate it**. Anticipated future consumers justify designing a seam, not
building the shared block.

## Problem: Duplication Across Three CUDA Modules

All CUDA code lives under `dart/simulation/experimental/compute/cuda/` as three
solver-specific modules, each a `.cu`/`.cuh` (+ `.cpp` host wrapper) triple:

| Module                           | Solver                                     | Plan     |
| -------------------------------- | ------------------------------------------ | -------- |
| `rigid_body_state_batch_cuda`    | Phase-5 homogeneous-batch rigid integrate  | PLAN-030 |
| `vbd_block_descent_cuda`         | Vertex Block Descent (mass-spring + tet)   | PLAN-104 |
| `deformable_psd_projection_cuda` | IPC/PD-IPC projected-Newton PSD projection | PLAN-081 |

The duplication is already concrete at N=3 (verified by source inspection):

| Duplicated concern                                       | Evidence                                                                                                                                                                                                                                                                                                                                                                    | Verdict                            |
| -------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- |
| Runtime/device probe `isCudaRuntimeAvailable()`          | Three identical declarations (`rigid_body_state_batch_cuda.cuh:45`, `deformable_psd_projection_cuda.cuh:52`, `vbd_block_descent_cuda.cuh:45`), one definition (`rigid_body_state_batch_cuda.cpp:229`), and a fragile cross-translation-unit "defined elsewhere" comment (`deformable_psd_projection_cuda.cpp:202`).                                                         | **Extract now**                    |
| CUDA-error-to-exception helper `throwIfCudaError`        | `rigid_body_state_batch_cuda.cpp:72` and `deformable_psd_projection_cuda.cpp:58` are byte-identical (`DART_EXPERIMENTAL_THROW_T` + `sx::InvalidOperationException`); `vbd_block_descent_cuda.cu:437` **diverges** to `throw std::runtime_error` (`:440`).                                                                                                                   | **Extract now**                    |
| Launch config (ceil-div grid) + `cudaGetLastError` check | Hand-rolled at 7+ sites across all three modules (e.g. `rigid_body_state_batch_cuda.cu:169,200`; VBD/PSD use block size 128 vs rigid's 256).                                                                                                                                                                                                                                | **Extract now**                    |
| Owning device buffer (`cudaMalloc`/`Memcpy`/`Free`)      | Three classes wrapping the same lifecycle: `DeviceDoubleBuffer` (`rigid_body_state_batch_cuda.cpp:85`), `DeviceArray<T>` (`vbd_block_descent_cuda.cu:448`), `ResidentDeviceBuffer` (`deformable_psd_projection_cuda.cpp:77`).                                                                                                                                               | **Extract now**                    |
| Divergent CPU/GPU integration math                       | `__device__ integrateOrientation` (`rigid_body_state_batch_cuda.cu:67`, free `sin/cos/sqrt`) is a re-derivation of `integrateOrientationsSemiImplicit` (`rigid_body_integration_kernel.hpp:113`), which is already `Scalar`-templated, Eigen-free, pointer-based, and documents itself as device-mappable but uses `using std::sin/cos/sqrt`. The two have already drifted. | **Single-source now**              |
| Upload-once / launch-many / download-once rollout        | `vbd_block_descent_cuda.cu` (device-resident rollout) vs `rigid_body_state_batch_cuda.cpp` (bare default-stream loop).                                                                                                                                                                                                                                                      | **One real consumer (VBD); defer** |
| CUDA-graph capture + replay                              | `vbd_block_descent_cuda.cu:512-522` only.                                                                                                                                                                                                                                                                                                                                   | **One consumer; defer**            |
| Mixed/selectable precision (float vs double)             | `vbd_block_descent_cuda.cuh:120` only.                                                                                                                                                                                                                                                                                                                                      | **One consumer; defer**            |
| Graph-coloring / CSR colored-sweep scheduling            | `vbd_block_descent_cuda.cu` only.                                                                                                                                                                                                                                                                                                                                           | **One consumer; defer**            |
| Backend-install registrar                                | `deformable_psd_backend.hpp` (PSD only).                                                                                                                                                                                                                                                                                                                                    | **One consumer; reuse by example** |

## Decision

**Build the minimal device-runtime substrate, single-source the one kernel that
has already drifted, and defer everything else behind a documented second-use
trigger.** This is the smallest change that removes the duplication that exists
_today_ without standing up speculative abstraction.

Concretely:

1. **Build now** (each justified by ≥2 of the three current modules): a single
   device-runtime header/source (`cuda_runtime.cuh/.cpp`), a single owning device
   buffer (`device_buffer.cuh`), and a single-sourced rigid integration kernel.
2. **Reuse, do not re-abstract**: the existing backend-neutral
   `deformable_psd_backend.hpp` registrar stays the device-attach seam, unchanged.
3. **Defer behind a seam**: rollout, CUDA-graph capture, precision policy,
   colored-sweep scheduling, a generic accelerator registry, a device IPC
   barrier/distance mirror, a per-`World` residency owner, and device
   reductions/A-Jacobi. Each has zero or one current consumer; each is extracted
   only when a real second consumer lands (see triggers below).

This complements — and does not fork — PLAN-083's CPU Newton-barrier
consolidation. The substrate owns only the GPU **device-resource** concern. When
device IPC math is eventually needed, the `detail/newton_barrier` scalar cores
become host/device-portable in place (PLAN-083 promote-on-second-use), never a
parallel GPU copy.

## What We Build Now

### L0 — `compute/cuda/cuda_runtime.cuh` (+ `cuda_runtime.cpp`)

The definitions live in a host `.cpp` (compiled by the host compiler like the
existing cuda `.cpp`s, not nvcc) so the single `throwIfCudaError` can use
`DART_EXPERIMENTAL_THROW_T` (`std::format` + `std::source_location`) without every
caller's `.cu` having to compile it. A flat set of free functions, not a
framework:

- `isCudaRuntimeAvailable()` — one `[[nodiscard]] bool ... noexcept` declaration
  and one definition (moved out of `rigid_body_state_batch_cuda.cpp:229`),
  deleting the two duplicate declarations and the cross-TU comment.
- `throwIfCudaError(cudaError_t, std::string_view)` — one definition on
  `DART_EXPERIMENTAL_THROW_T` + `sx::InvalidOperationException`, replacing the
  byte-identical rigid/PSD pair and the divergent VBD `std::runtime_error`.
- `launchGrid1D(std::size_t n, unsigned blockSize)` — ceil-div helper that takes
  block size **per call** (rigid keeps 256, PSD/VBD keep 128; the helper imposes
  no single block size).
- `checkLastError(std::string_view)` — wraps `cudaGetLastError()`.

Kept as plain functions — explicitly **not** a configurable error-policy or
launch framework.

### L0 — `compute/cuda/device_buffer.cuh`

One thin RAII `template <typename T = double> class DeviceBuffer` subsuming
`DeviceDoubleBuffer`, `DeviceArray<T>`, and `ResidentDeviceBuffer`:
`cudaMalloc` on construct, `copyToDevice`/`copyFromDevice` with one consistent
zero-count guard, `cudaFree` in the destructor, deleted copy. Default `T=double`
preserves Phase-5 parity exactly; `float` is opt-in only. The resident
grow-and-reuse behavior unique to PSD (avoid per-call `cudaMalloc`/`cudaFree`,
plus an `allocationCount()` for the test that asserts it) is an explicit opt-in
method (`ensure(count)`), not a default mode — kept as a thin wrapper rather than
a mode flag if a flag would bloat the base type.

### L1 — single-source the rigid integration kernel

Extract the drifted math — the quaternion exponential-map orientation update —
into a per-body `__host__ __device__` template core
(`integrateOrientationSemiImplicitBody`) in
`compute/detail/rigid_integration_core.hpp`. The CPU batch function
`integrateOrientationsSemiImplicit` (`rigid_body_integration_kernel.hpp`) loops
calling that core, and `rigid_body_state_batch_cuda.cu`'s `__global__` kernel
calls the same core per thread, after which the divergent `__device__
integrateOrientation` (`:67`, free `sin/cos/sqrt` vs the CPU `using std::`) is
**deleted**. This removes the most dangerous duplication class — CPU/GPU math
that has already drifted — and serves the single-source-of-truth axiom (#5). The
trivial position/velocity updates are not drift-prone (`positions[i] +=
velocities[i] * dt`), so they stay as-is; only the orientation core is single-
sourced.

The portability macro `DART_EXPERIMENTAL_HD` lives in the same `detail/` header
(`compute/detail/`), **not** `dart/common`: its only consumers are two
experimental modules, so per the surgical axiom (#4) it must not widen the blast
radius of the stable core. It is guarded by `__CUDACC__` (expanding to
`__host__ __device__` under nvcc and to nothing under the host compiler) and its
spelling carries no backend token. A `detail/` header is excluded from Doxygen
and skipped by the compute-backend boundary scanner, so the device qualifiers
never reach a scanned public header — while the public installed
`rigid_body_integration_kernel.hpp` gains only an `#include` and a `detail::`
call, staying boundary-clean.

### Reused unchanged — `compute/deformable_psd_backend.hpp`

The capability-named function-pointer registrar that lets the deformable solver
offload PSD projection to an installed backend is already backend-neutral and is
the only device-attach seam touching the core. It is reused **by example**; a
generic `accelerator_registry.hpp` is **not** extracted (one registrar consumer).

## What We Defer (Extract on Documented Second Use)

Each deferred item gets a designed seam now and a written trigger; none is built
until the trigger fires. Building any of them now would violate axiom #3.

| Deferred block                                 | Current consumers | Extraction trigger                                                                                                                                                                                                                          |
| ---------------------------------------------- | ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `device_rollout.cuh` (resident multi-step)     | 1 (VBD)           | AVBD-CUDA (PLAN-104) or Phase-6 rigid (PLAN-030) needs the identical upload-once/replay shape.                                                                                                                                              |
| CUDA-graph capture + replay helper             | 1 (VBD)           | A second solver wants graph amortization; or shipped on its own as a reviewed rigid change with parity + benchmark.                                                                                                                         |
| Selectable-precision policy                    | 1 (VBD)           | A second solver adopts mixed precision.                                                                                                                                                                                                     |
| Colored-sweep / CSR launch scheduler           | 1 (VBD)           | AVBD-CUDA becomes the second colored-sweep consumer.                                                                                                                                                                                        |
| Generic `accelerator_registry.hpp`             | 1 (PSD)           | rigid-IPC GPU (PLAN-082) or AVBD-CUDA becomes the second registrar.                                                                                                                                                                         |
| Device IPC barrier/distance/CCD/tangent mirror | 0                 | First GPU IPC consumer (PLAN-082/081) lands → make the **same** `detail/newton_barrier` scalar cores host/device-portable (PLAN-083), never a copy. The `Eigen` `Vector12d`/`Matrix12d` types there are the genuine Eigen-on-device hazard. |
| Per-`World` device-residency owner             | 0 cross-stage     | A concrete 2-stage GPU pipeline measurably pays the host round-trip (build fully behind a pimpl).                                                                                                                                           |
| Device reductions / A-Jacobi / sparse solve    | 0                 | PD-IPC GPU CPU-verifiable slice lands; reductions must be designed against the fixed-merge-order determinism rule.                                                                                                                          |

If rigid CUDA-graph amortization is wanted before its trigger fires, it ships as
its own separately-reviewed change with before/after parity and a benchmark —
never folded into a dedup commit as a "free" behavior change.

## Library Verdicts

The maintainer goal asks whether to stand up dedicated libraries for common math,
collision, numerical optimization, data structures, and device runtime. The
evidence-backed verdicts:

| Candidate library                                               | Verdict                     | Reason                                                                                                                                                                                                                                                                                                                                 |
| --------------------------------------------------------------- | --------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Common math (`dart/math`: `lie_group`, geometry, optimization)  | **Reuse existing**          | `dart/math` is the established CPU math home. The only math single-sourcing justified now is annotating the existing `rigid_body_integration_kernel.hpp` host/device-callable for its two current consumers. Neo-Hookean/eigensolve/barrier device math is deferred (Eigen-on-device unproven under `nvcc`, <2 current GPU consumers). |
| Collision / IPC primitives (`detail/newton_barrier`)            | **Defer**                   | `detail/newton_barrier` is the canonical CPU IPC owner under PLAN-083 with ≥3 CPU consumers but **zero** `__device__` annotations and zero current GPU consumers. A device mirror now would violate axiom #3 and hit the Eigen-on-device hazard. Promote the same cores on first GPU IPC use, per PLAN-083.                            |
| Numerical optimization (`dart/optimizer`, `dart/math/lcp`, PSD) | **Reuse existing**          | PSD-cone projection already has the correct backend-neutral function-pointer registrar (`deformable_psd_backend.hpp`) with a CUDA sidecar. Reuse it unchanged; do not extract a generic registry (one consumer). A-Jacobi / device sparse solve deferred (0 consumers).                                                                |
| Data structure (SoA state, CSR/coloring, device buffers)        | **Build now (buffer only)** | The one data structure with proven N=3 duplication is the owning device buffer → unify into `DeviceBuffer<T>`. The SoA `RigidBodyStateBatch`/`Model` layout is already shared and unchanged. VBD's CSR/coloring stays VBD-local (one consumer).                                                                                        |
| Device runtime (probe + error mapping + launch config)          | **Build now**               | Highest-certainty, zero-risk extraction (triplicated probe, three error helpers with one divergent, 7+ hand-rolled launch sites). Consolidate into `cuda_runtime.cuh/.cpp`.                                                                                                                                                            |

The headline: **no new general-purpose library is created.** The shared layer is
a thin device-runtime substrate plus one buffer type plus one kernel
single-sourcing — and existing CPU homes (`dart/math`, `dart/optimizer`,
`detail/newton_barrier`, `deformable_psd_backend`) are reused, not duplicated for
the GPU.

## Directory Layout

```text
dart/simulation/experimental/compute/
  rigid_body_integration_kernel.hpp        # EDIT: orientation batch loop calls the shared per-body core (installed .hpp, backend-token-free)
  rigid_body_state_batch.{hpp,cpp}         # unchanged SoA State/Model
  deformable_psd_backend.{hpp,cpp}         # UNCHANGED: the only public device-attach seam, reused by example
  compute_stage_metadata.{hpp,cpp}         # unchanged (the reserved ComputeStageAcceleration::Gpu bit stays inert)
  detail/
    rigid_integration_core.hpp             # NEW (L1): DART_EXPERIMENTAL_HD macro + per-body __host__ __device__ orientation core (detail/ = scanner-skipped, Doxygen-excluded)
  cuda/                                    # all shared device code is .cuh/.cpp — .cuh never installed/doxygen'd/boundary-scanned; .cpp under cuda/ not scanned
    cuda_runtime.cuh                       # NEW (L0): isCudaRuntimeAvailable decl + throwIfCudaError/checkLastError decls + inline launchGrid1D
    cuda_runtime.cpp                       # NEW (L0): the single isCudaRuntimeAvailable + throwIfCudaError + checkLastError definitions (host TU)
    device_buffer.cuh                      # NEW (L0): template<T=double> DeviceBuffer RAII + opt-in resident ensure()/allocationCount()
    rigid_body_state_batch_cuda.{cuh,cu,cpp}     # REFACTOR onto L0; kernel calls the L1 orientation core; DELETED the divergent integrateOrientation
    deformable_psd_projection_cuda.{cuh,cu,cpp}  # REFACTOR onto L0 (DeviceBuffer resident scratch); registers via the UNCHANGED psd_backend
    vbd_block_descent_cuda.{cuh,cu}              # REFACTOR onto L0 (DeviceBuffer<T> for double+float); unified the throw idiom; rollout/coloring stay LOCAL

# NOT created now (documented seams; no header, no directory, no scaffolding):
#   device_rollout.cuh · colored_sweep.cuh · compute/accelerator_registry.hpp ·
#   standalone device_math.cuh · device newton_barrier mirror · residency owner ·
#   A-Jacobi/reduction primitive. The macro is NOT placed in dart/common.
```

CMake: add `cuda_runtime.cpp` to the existing `${target_name}-cuda` STATIC
library source list (the `.cuh`/header-only files need no listing)
(`dart/simulation/experimental/CMakeLists.txt`). It stays gated on
`DART_ENABLE_EXPERIMENTAL_CUDA` (default OFF) and remains build-only. No new
install/export rule; no GPU package added to any default Pixi feature or wheel
manifest.

## How the Constraints Are Satisfied by Construction

The hard guardrails from `scalable_compute_decisions.md` and
`docs/ai/principles.md` hold without new enforcement, all verified in-tree:

- **No public-API leakage.** Shared device code lives in `.cuh`/`.cu` under
  `compute/cuda/`. The boundary scanner only reads `{.cpp, .h, .hpp}`
  (`scripts/check_compute_backend_boundaries.py:25`), so `.cuh`/`.cu` are out of
  scope; the one edited installed header (`rigid_body_integration_kernel.hpp`)
  gains only a backend-token-free macro that expands to empty in host builds, so
  it stays clean to `check-api-boundaries` and `check-compute-backend-boundaries`.
- **GPU is an optional sidecar.** The `-cuda` STATIC library is **not** passed to
  `add_component_targets` (only the base target is, at
  `dart/simulation/experimental/CMakeLists.txt:203`), so it is never installed,
  exported, linked into the default `dart` target, the default Pixi environment,
  or the official `dartpy` wheel. `check-no-gpu-runtime-dependencies` continues to
  guard the default manifests. The CPU fallback with identical semantics is
  preserved (every refactor is behavior-preserving; PSD's CPU reference and the
  rigid CPU kernel remain the canonical paths).
- **Determinism / parity.** `double` stays the default in `DeviceBuffer<T>`;
  single-sourcing the rigid kernel can only tighten CPU/GPU parity (both paths run
  one body of code). Float and any future reductions are opt-in and gated by the
  tolerance/fixed-ULP contract.
- **Backend reversibility.** Nothing names CUDA in a public type or namespace; the
  substrate is internal device plumbing behind the existing executor/registrar
  seams, so the CUDA-vs-SYCL choice stays reversible (`--expt-relaxed-constexpr`
  is already set at `CMakeLists.txt:181`).

## Phased Roadmap

The build-now sequence below is complete on `main` in PR #2875. Future work uses
the deferral table above and must promote only real second-use extractions.
Canonical priority, next step, and gate live in PLAN-031.

- **P0 — pin parity baselines (complete).** Record CPU/GPU parity for all three
  modules at the existing tolerance (Phase-5 reference 1.78e-15) before any
  extraction, and confirm the default no-GPU build + `dartpy` import are
  unchanged. Exit evidence: baselines committed; all boundary/packaging checks
  green as the starting line.
- **P1 — `cuda_runtime.cuh/.cpp` + refactor all three modules onto it
  (complete).** Exit evidence: the three module parity tests stay green within
  recorded tolerance; `.cuh`-only confirmed (no `.hpp` gains a backend token);
  the `-cuda` library stays build-only.
- **P1b — VBD error-type unification (complete).** Moving VBD's
  `std::runtime_error` onto `DART_EXPERIMENTAL_THROW_T` changes the thrown type at
  the `World::step` boundary, so it ships as its own commit with an explicit
  PR-body note and any downstream catcher updated in the same PR.
- **P2 — `device_buffer.cuh` + migrate the three buffer classes (complete).**
  Exit evidence: outputs identical pre/post; PSD's `allocationCount()` invariant
  preserved; no benchmark regression.
- **P3 — single-source the rigid integration kernel (complete).** Exit evidence:
  rigid CPU/GPU parity passes (and ideally tightens); the macro expands to empty
  in the default no-CUDA build; boundary checks green.
- **Future — extract on documented second use.** Per the deferral table; each
  extraction requires its real second consumer, a separate review, a
  pre-extraction parity pin, and an unchanged public-API/sidecar guarantee.

## Resolved Build-Now Questions

PR #2875 resolved the questions that affected the landed substrate:

- `DeviceBuffer<T>` uses opt-in `ensure(count)` / `allocationCount()` resident
  reuse inside the shared type, preserving the PSD invariant without adding a
  second buffer wrapper.
- The host/device annotation lives in the experimental-scoped
  `compute/detail/rigid_integration_core.hpp` header as `DART_EXPERIMENTAL_HD`,
  expanding to `__host__ __device__` only under `nvcc` and to empty under host
  builds.
- The VBD error-type divergence is unified onto the shared CUDA error helper and
  `InvalidOperationException` path.

## Deferred Questions

- Rigid CUDA-graph amortization: does the maintainer want it as a standalone
  reviewed change soon, or left until the rollout helper's second consumer
  naturally triggers extraction?
- PSD eigensolve canonicalization: should the CPU Eigen self-adjoint solve and
  the GPU cyclic-Jacobi ever be unified to one algorithm under the fixed-ULP
  parity contract, or is the documented numerical divergence acceptable? Currently
  deferred.
- Parity-test coverage: do rigid and VBD have CUDA parity tests at PSD's maturity
  to pin behavior across P1–P3, or must new parity tests be authored in P0?

## Related Designs

- [`scalable_compute_decisions.md`](scalable_compute_decisions.md) owns the GPU
  packaging shape, prototype gate, determinism contract, and CUDA-vs-SYCL
  criteria this substrate operates within.
- [`compute_backend_research.md`](compute_backend_research.md) owns the workload
  evidence and the Newton-style immutable-Model / batched-SoA-State direction the
  device buffer and residency seam anticipate.
- [`simulation_solver_architecture.md`](simulation_solver_architecture.md) owns
  the experimental solver/stage/executor architecture the CUDA modules attach to.
- PLAN-083 (`083-unified-newton-barrier-multibody.md`) owns CPU Newton-barrier
  primitive consolidation; the deferred device IPC math promotes those same
  `detail/newton_barrier` cores host/device-portable rather than copying them.
- `docs/plans/dashboard.md` PLAN-031 owns this work's operating state; PLAN-030,
  PLAN-104, PLAN-082, and PLAN-081 are the dependent GPU-solver tracks this
  substrate de-duplicates.
