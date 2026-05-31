# PLAN-081 PD-IPC GPU Gap Audit

- Operating state: `PLAN-081` in [`../dashboard.md`](../dashboard.md)
- Owner plan:
  [`../081-deformable-implicit-barrier-solver.md`](../081-deformable-implicit-barrier-solver.md)
- Purpose: plan the research and implementation path for Penetration-free
  Projective Dynamics on the GPU (PD-IPC) as a DART-owned GPU-accelerated
  deformable contact solver path.

## Source Evidence

- Paper: Lan, Ma, Yang, Zheng, Li, and Jiang, "Penetration-free Projective
  Dynamics on the GPU," ACM Transactions on Graphics 41(4), Article 69, 2022.
  The paper combines projective dynamics with IPC-style barriers, a two-level
  projective IPC iteration, GPU patch-based collision culling, minimum-gradient
  Newton CCD pruning, and an aggregated Jacobi (A-Jacobi) global solve.
- The paper's guarantee is conditional: intersection-free trajectories are
  claimed when deformable bodies start apart, and CCD pruning keeps each outer
  loop feasible. DART must not use PD-IPC as recovery evidence for already
  self-intersecting states; that remains the SPB sidecar's role.
- Public talk/slides: the GAMES webinar deck records the same decomposition
  into two-level projective IPC, A-Jacobi, patch-based GPU collision culling,
  and comparisons against CPU IPC on rubber-helicopter and armadillo scenes.
- Reference CCD implementation:
  `Continuous-Collision-Detection/Fast-Approximate-Root-CCD` at HEAD
  `ecf87abbbf6b7c5ebdb946a8f4883021455aab10`, MIT licensed. Its README says
  the implementation is faithful to the paper description but can produce false
  negatives for both point-triangle and edge-edge queries. Treat this as a
  validation and risk source, not as an implementation to vendor or trust for
  production CCD.
- Robotics projective-dynamics reference:
  `DragonZoom/projective-dynamics-for-robotics` at HEAD
  `e0671d78253eea580f558a0e6ea5e08942f488c9`, Apache-2.0 licensed. Its README
  describes an early ROS2 package for real-time projective dynamics of
  deformable robot parts, with only simple visualization available. Treat it as
  robotics use-case evidence, not as a DART runtime dependency.

## DART Placement

PD-IPC belongs under PLAN-081 first. It is an IPC-class deformable contact path
whose acceleration story depends on GPU data layout and solver kernels. It
should reuse the PLAN-081 mesh/material/contact foundation and the PLAN-030
compute boundary rather than creating a public GPU solver family. PLAN-104/VBD
and OGC are comparison baselines because they already carry GPU-shaped
deformable/contact evidence, but PD-IPC should not be framed as VBD or OGC.

The first DART implementation should be internal and CPU-verifiable before any
GPU claim. Public `World` and dartpy APIs must stay algorithm- and
backend-neutral. Build flags, benchmarks, diagnostics, and developer docs may
say CUDA or GPU when needed, but no public type, namespace, solver selector, or
resource handle should expose PD-IPC, A-Jacobi, CUDA streams, kernels, memory
pools, ROS2, or reference-project names.

## Method Responsibilities To Audit

| Area                           | PD-IPC responsibility                                                                                                                                                       | DART starting point                                                                                           | Status  |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | ------- |
| Mesh/material state            | Fullspace deformable solids and shells with projective local constraints, inertia, boundary conditions, and surface topology for contact.                                   | PLAN-081 mesh/material state, scene loading, IPC gap audit, and VBD elasticity kernels.                       | Planned |
| Projective IPC iteration       | Two-level outer/inner loop that rebuilds barrier landmarks at outer iterations, runs local/global PD inner iterations, and applies CCD pruning after the inner solve.       | PLAN-081 IPC solver loop, barrier stiffness adaptation, and diagnostics are not complete yet.                 | Planned |
| Contact and barrier projection | IPC-style PT/EE barriers, target-position construction, finite clearance, and candidate filtering for self-contact and inter-body contact.                                  | PLAN-081 PT/EE derivative work, conservative CCD line search, and SPB recovery sidecar.                       | Planned |
| Fast CCD pruning               | Minimum-gradient Newton approximate root for cubic vertex-triangle and edge-edge CCD, compared against conservative/exact DART CCD.                                         | PR #2700 native primitive CCD and validation-only exact/root-safe query paths.                                | Planned |
| GPU collision culling          | Patch-based GPU culling over dynamic surface primitives with deterministic candidate ordering and CPU fallback.                                                             | Native collision BVH/query foundations and PLAN-030 compute executor metadata.                                | Planned |
| A-Jacobi global solve          | Aggregated Jacobi products, weighted variants, Chebyshev acceleration, sparsity suppression, and shared-memory/blocking strategy.                                           | VBD GPU/CUDA benchmark harness, compute-backend boundary checks, and internal solver diagnostics.             | Planned |
| Corpus and benchmarks          | Table/figure scenes such as rubber helicopters, flattened armadillo, dragon, tiered skirt, animal-crossing, and Halloween-party workloads, scaled into DART-owned fixtures. | PLAN-081 figure showcase, benchmark JSON conventions, headless Filament capture path, and CUDA packet checks. | Planned |

## Implementation Sequence

1. **Source and equation audit** - Pin the paper PDF, GAMES deck, video, CCD
   repository commit, and robotics repository commit. Map Algorithm 1, barrier
   projection, CCD pruning, A-Jacobi equations, GPU culling, parameters, and
   paper Table/Figure workloads to DART-owned row IDs.
2. **CPU-verifiable projective IPC slice** - Add an internal CPU reference for
   the two-level projective IPC iteration on a tiny mesh. This slice should use
   DART's conservative CCD and exact validation queries first, so correctness
   evidence is independent of the paper's approximate root shortcut.
3. **Fast-CCD validation slice** - Reproduce the paper's minimum-gradient
   Newton CCD and test it against DART conservative CCD on generated and
   adversarial PT/EE cases, including the public false-negative examples from
   `Fast-Approximate-Root-CCD`. Promotion requires zero missed collisions in the
   accepted domain or a documented fallback to conservative CCD.
4. **A-Jacobi CPU/GPU prototype slice** - Implement A-Jacobi behind internal
   solver names with a CPU fallback and optional CUDA sidecar. Tests compare
   residual decrease, Chebyshev compatibility, deterministic output, and
   convergence against Jacobi, colored Gauss-Seidel/VBD, and PCG baselines on
   the same systems.
5. **GPU culling and data-layout slice** - Add patch-based culling and
   device-resident contact buffers only behind PLAN-030 private compute
   boundaries. Evidence must include transfer/setup/kernel/readback timing and
   no default-package GPU runtime dependency.
6. **Corpus, benchmark, and visual slice** - Add reduced DART-owned versions of
   the paper scenes with invariants, benchmark/profiling JSON, headless Filament
   captures, and same-host CPU/GPU packets. At least one shell scene and one
   volumetric solid scene must pass before any broad PD-IPC claim.
7. **Promotion decision** - Promote PD-IPC from evaluation only if it beats the
   CPU IPC and VBD/OGC baselines on the same corpus without weakening DART's
   conservative CCD/no-intersection contract or leaking backend details.

## Acceptance Before Promotion

PD-IPC progress is not promotable from evaluation to an active implementation
claim until DART has:

- a completed paper/deck/video/code matrix with pinned source versions and row
  IDs for every adopted equation, algorithm, parameter, and workload;
- CPU reference tests for projective IPC outer/inner iteration, barrier
  projection, local/global PD energy decrease, and CCD pruning semantics;
- PT/EE distance, derivative, barrier, and candidate-filtering tests shared
  with the broader PLAN-081 IPC path;
- fast-CCD validation against DART conservative CCD, including adversarial
  false-negative examples and a fallback policy whenever approximate roots are
  unsafe;
- A-Jacobi residual, convergence, Chebyshev, deterministic-reduction, and
  same-system baseline tests against Jacobi, VBD/colored Gauss-Seidel, and PCG;
- optional GPU culling and A-Jacobi benchmarks that include setup, transfers,
  kernel time, and readback, plus CPU fallback parity on the same input;
- benchmark/profiling JSON for table/figure workloads and same-host CPU/GPU
  packets that satisfy PLAN-030 GPU packet conventions before any speedup claim;
- headless Filament evidence for every paper-inspired scene used in the claim;
  and
- `pixi run lint`, `pixi run build`, focused C++/CUDA tests,
  `pixi run check-api-boundaries`,
  `pixi run check-compute-backend-boundaries`, and
  `pixi run check-no-gpu-runtime-dependencies` green for every implementation
  slice.

## Non-Goals

- Do not vendor the paper assets, `Fast-Approximate-Root-CCD`, ROS2,
  `projective-dynamics-for-robotics`, OpenGL/GLFW/GLEW, or any reference
  dependency as a DART runtime dependency.
- Do not expose PD-IPC, A-Jacobi, reference repository names, solver registries,
  ECS storage, CUDA streams, kernels, memory pools, or backend/device resource
  types through public C++ or dartpy APIs.
- Do not replace DART's conservative CCD/no-intersection contract with the
  approximate CCD shortcut unless the accepted domain is proven against
  adversarial PT/EE tests.
- Do not claim support for already self-intersecting recovery, rigid impact, or
  broad GPU contact parity from a solver, CCD, culling, or single-scene slice.
