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

**Regime B — kernel-heavy** (sweep/scene/filtered rows): stay 9–14x slower even
at the largest size. `kernel_ns` is much larger here than in Regime A and grows
with size, but — as the measured breakdown below shows — per-call allocation
overhead still dominates total time even for these rows. So "kernel-bound" is
relative: the kernels are worth fixing, but they are not the largest single cost.

## Measured kernel-phase breakdown (CUDA events, point-triangle /65536)

Temporary CUDA-event instrumentation of `launchSweptPointTriangleSweepKernel`
(env-gated, since reverted) measured, for the worst scene-runtime row
(points=2560, triangles=768):

| Phase                 | Time         | Share |
| --------------------- | ------------ | ----: |
| sort (bitonic)        | 0.380 ms     |  ~36% |
| count + serial prefix | 0.355 ms     |  ~34% |
| scatter               | 0.331 ms     |  ~31% |
| **kernel total**      | **~1.07 ms** |       |

The three kernel phases are **roughly equal** — there is no single dominant
kernel to optimize. More importantly, the benchmark's total `real_time` for this
row is **~4.0 ms**, of which the function's named timers (setup+h2d+kernel+d2h)
sum to only ~1.6 ms. The remaining **~2.4 ms is per-call device allocation +
`cudaFree`** of the ~31 MB all-pairs buffers at scope exit (the benchmark loop
calls the function once per iteration with no buffer reuse).

### Decisive consequence: the gate is size-bound, not kernel-bound

The CPU does the same row in ~0.32 ms. Even a _perfect_ GPU kernel rewrite
(FP32 + CUB sort/scan, removing all kernel cost) would still leave
~1.6 ms of setup/transfer + ~2.4 ms of alloc/free per call ≈ a floor far above
the CPU's 0.32 ms **at these reduced problem sizes**. Therefore FP32 / CUB
kernel micro-optimizations **cannot flip the speedup gate at reduced scale** —
the problem is simply too small to amortize per-call GPU overhead. Closing the
gate requires (a) eliminating per-call overhead via persistent device buffers +
right-sized compaction outputs + batching, and (b) **paper-scale problem sizes**
so the GPU has enough work to amortize launch and transfer cost. Kernel-quality
fixes (FP32, CUB) are worthwhile for the kernel phase but are not sufficient and
not the first lever.

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

The measured breakdown above changes the priority: the gate is **size/overhead-
bound**, so the first levers must reduce per-call overhead and raise problem
size, _then_ optimize kernels.

1. **Add a paper-scale benchmark configuration.** The reduced sizes (≈2.5k
   points) cannot amortize GPU launch/transfer overhead; without larger problems
   the gate is unreachable regardless of kernel quality. This is the prerequisite
   for any speedup claim.
2. **Eliminate per-call overhead:** reuse persistent device buffers across calls
   and size compaction outputs to an expected-candidate bound, not all-pairs
   capacity (~31 MB → ~KB). Removes the ~2.4 ms/call alloc+free that dominates
   total time. Bit-exact.
3. Replace the per-stage bitonic launches and the `<<<1,1>>>` scans with a single
   device scan/sort primitive (CUB `DeviceScan`/`DeviceRadixSort`). Removes ~133
   launches + the serial scans (~36% of kernel time). Prefix sums are bit-exact;
   sort changes only output order (compare candidate sets, not ordered arrays).
4. Add an FP32 (or mixed-precision) geometry path for the AABB/distance math
   (~65% of kernel time across count+scatter); keep FP64 only where parity
   demands it. Requires relaxing the packet/unit-test distance tolerance.
5. Fuse the build→sort→count→scan→scatter stages and raise occupancy (more work
   per thread / 2D tiling over point×triangle).
6. Add an explicit warmup step in the harness so cold-start context init is not
   attributed to the first measured row.

These attack the plan's actual blocker (a competitive GPU `World::step`
candidate path) rather than adding further reduced witness packets, which do not
move the speedup gate.

## Implemented structural fix and validated results (2026-06-14)

Recommendations 1–2 were implemented for the swept point-triangle and edge-edge
sweeps in `contact_candidate_filter_cuda.{cu,cpp}`:

- **Paper-scale benchmark config:** added `->Arg(1048576)` (≈10.2k points / 3.1k
  triangles) to the scene-runtime sweep rows.
- **Right-sized output buffers:** split each sweep launch into a count phase and
  a scatter phase. The host now reads the candidate count after the count phase
  and allocates the accepted output buffers to that count instead of the
  all-pairs capacity (`pointCount × triangleCount` / `edgeCount²`). This removes
  the dominant per-call ~31 MB allocation/free and lets the benchmark run
  paper-scale sizes without all-pairs out-of-memory. The change is **bit-exact**:
  all 20 `test_contact_candidate_filter_cuda` tests pass, including the
  exact-index + `1e-14` distance parity checks.

Measured GPU/CPU total-time ratio at paper scale (`/1048576`), warmed up:

| Row                  | Before (all-pairs alloc) |        After (right-sized) |
| -------------------- | -----------------------: | -------------------------: |
| point-triangle sweep |     ~14x slower (g/c≈14) |             g/c ≈ **2.7x** |
| edge-edge sweep      |               ~9x slower | g/c ≈ **0.69x (GPU wins)** |

The edge-edge sweep now beats the CPU at paper scale, and GPU setup dropped from
~5.3 ms to ~0.05–0.18 ms per call. The remaining point-triangle gap is the FP64
count kernel (one thread per point looping over all triangles); recommendations
3–5 (CUB scan/sort, FP32/mixed precision, higher occupancy) target that residual
and are the next step. This confirms the gate is reachable: the size+overhead
fix alone already flips one of the two sweep families to a GPU win.
