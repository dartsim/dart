# PLAN-083 GPU Contact-Candidate Slowness — Root-Cause Analysis

Date: 2026-06-14. Hardware: NVIDIA RTX 5000 Ada **Laptop** GPU (Ada AD10x,
FP64:FP32 = 1:64), CUDA 12.4. Binary: `bm_plan083_gpu_contact_candidates`
(Release). This analysis explains why the private GPU contact-candidate parity
packets report sub-1x speedups (`meets_speedup_gate=false`, typically 0.02x–0.10x
on the sweep/scene rows) and is intended to redirect effort from accumulating
more reduced evidence packets toward the actual blockers.

## Method

1. Read the per-row `timing_ns` + `speedup` already recorded in
   `gpu-parity-packet.json`.
2. Ran the benchmark across problem sizes (`/4096`, `/65536`) and compared the
   GPU/CPU real-time ratio and `kernel_ns` per family.
3. Read the CUDA implementation in
   `dart/simulation/compute/cuda/contact_candidate_filter_cuda.{cu,cpp}`.
4. Attempted hardware profiling: `nvprof` is unsupported on Ada (CC 8.9); `nsys`
   stats need a report importer that is missing in this environment; `ncu`
   returns `ERR_NVGPUCTRPERM` (needs admin perf-counter access). Per-kernel
   attribution below is therefore from source structure + scaling behavior, not
   a hardware profile.

## Two distinct regimes

| Family (median row)                 | /4096 g/c |           /65536 g/c | dominant GPU cost |
| ----------------------------------- | --------: | -------------------: | ----------------- |
| PointTriangleCandidateMask          |  ~1691x\* | **0.77x** (GPU wins) | per-call overhead |
| SweptPointTriangleCandidateMask     |     7.55x |            **0.89x** | per-call overhead |
| SceneRuntimePointTriangleSweep      |     27.5x |                14.4x | kernel time       |
| SceneRuntimeEdgeEdgeSweep           |     44.6x |                 9.3x | kernel time       |
| SceneRuntimeFilteredCandidateBuffer |     26.6x |                10.2x | kernel time       |

`*` The first GPU benchmark to run absorbs one-time CUDA context init
(~120–200 ms); it is an ordering artifact, not steady-state cost.

**Regime A — overhead-bound** (mask/buffer rows): reach parity or win at 65536.
The GPU compute is fine; small problems are dominated by fixed per-call cost.

**Regime B — kernel-bound** (sweep/scene/filtered rows): stay 9–14x slower even
at the largest size, with `kernel_ns` dominating and growing. The kernels
themselves are the bottleneck.

## Root causes (ranked)

### Kernel-bound rows (Regime B)

1. **Bitonic sort issued as O(log²n) individual kernel launches.**
   `sortSweepItemsOnDevice` (`contact_candidate_filter_cuda.cu:1529`) nests
   `for stage` × `for stride` and launches `bitonicSortSweepItemsKernel<<<>>>`
   once per compare-exchange stage. For padded 4096 points that is ~78 launches,
   plus ~55 for 1024 triangles ≈ **~133 tiny kernel launches per iteration just
   to sort**, each paying kernel-launch latency. This is launch-latency-bound and
   is the most likely dominant kernel cost.
2. **Serial single-thread prefix sums.** `prefixSweepPairCountsKernel<<<1u,1u>>>`
   (`:1627`) and `prefixPointTriangleAcceptedBlockCountsKernel<<<1u,1u>>>`
   (`:1270`, `:1364`, `:1436`, `:1506`) run a scan on **one GPU thread** — zero
   parallelism, full serialization of the pipeline.
3. **FP64 everywhere on a 1:64 FP64 GPU.** Positions and distances are `double`
   throughout the kernels; on this Ada laptop part FP64 runs at 1/64 of FP32, so
   all geometry math pays up to a ~64x penalty versus an FP32/mixed path.
4. **Low occupancy.** Count/scatter launch one thread per point
   (`:1614`, `:1634`); the scene scenes have ~2,560 points → ~10 blocks of 256,
   leaving most of a ~58-SM GPU idle.

### Overhead-bound rows (Regime A)

5. **Per-call allocation, over-sized to all-pairs capacity.**
   `buildSweptPointTriangleSweepCuda` (`contact_candidate_filter_cuda.cpp:1051`)
   allocates ~12 `DeviceBuffer`s every call, several sized to
   `pairCapacity = pointCount × triangleCount` (~1.97M elements ≈ ~31 MB) even
   though only ~512 candidates survive. RAII frees them each iteration, so there
   is no cross-call reuse. This dominates `host_setup_ns` (≈0.3–1.1 ms/call).
6. **Cold-start CUDA context init** (~120–200 ms) lands on the first GPU
   benchmark and skews whichever row runs first.

## Why the speedup gate cannot be met as-is

The gate compares against a CPU sorted sweep-and-prune that runs the same scene
in tens of microseconds. The GPU path pays ~133 serial kernel launches + serial
scans + FP64 + ~31 MB allocation per call to process a few thousand points. At
these (reduced, non-paper-scale) sizes the GPU can never amortize that, so the
gate failure is **structural, not a measurement error**. It is, however,
fixable in principle.

## Recommendations (highest leverage first)

1. Replace the per-stage bitonic launches and the `<<<1,1>>>` scans with a
   single device scan/sort primitive (CUB `DeviceScan`/`DeviceRadixSort` or
   thrust). Removes ~133 launches + the serial scans in one change.
2. Add an FP32 (or mixed-precision) geometry path for the AABB/distance math;
   keep FP64 only where parity demands it. Largest single arithmetic win on this
   GPU class.
3. Reuse persistent device buffers across calls and size compaction outputs to
   an expected-candidate bound, not all-pairs capacity. Removes per-call
   `cudaMalloc`/over-allocation.
4. Fuse the build→sort→count→scan→scatter stages and raise occupancy (more work
   per thread / 2D tiling over point×triangle).
5. Add an explicit warmup step in the harness so cold-start context init is not
   attributed to the first measured row.

These attack the plan's actual blocker (a competitive GPU `World::step`
candidate path) rather than adding further reduced witness packets, which do not
move the speedup gate.
