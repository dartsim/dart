# PLAN-081 SPB Gap Audit

- Operating state: `PLAN-081` in [`../dashboard.md`](../dashboard.md)
- Owner plan:
  [`../081-deformable-implicit-barrier-solver.md`](../081-deformable-implicit-barrier-solver.md)
- Purpose: plan the research and implementation path for Shortest Path to
  Boundary for Self-Intersecting Meshes (SPB) as a DART-owned recovery and
  contact-direction method for volumetric deformable self-intersections.

## Source Evidence

- Paper/project: Chen, Diaz, and Yuksel, "Shortest Path to Boundary for
  Self-Intersecting Meshes," ACM Transactions on Graphics 42(4), 2023. The paper
  defines a shortest boundary path for self-intersecting meshes, then computes
  the exact shortest path from an internal point to a valid boundary point using
  candidate boundary points and robust tetrahedral traversal.
- ArXiv/project pages describe the target use case as collision and
  self-collision handling for deformable volumetric objects when the fast
  simulator does not guarantee complete collision resolution.
- Supplemental material provides pseudocode for tetrahedral traversal,
  exit-face selection, shortest-path query, and infeasible-region culling.
- Reference code:
  `AnkaChan/Shortest-Path-to-Boundary-for-Self-Intersecting-Meshes` at HEAD
  `a0a8440af4bda83340bcd102f7d3150245a86b36`. The code is Apache-2.0, tested
  with Windows/Visual Studio, and depends on MeshFrame2, CuMatrix, OneTBB,
  Eigen, and Embree. Treat it as method evidence; do not vendor its third-party
  stack or expose its project names through DART's public surface.

## DART Placement

SPB belongs under PLAN-081 first. It is not a replacement for IPC barriers, OGC,
or native rigid collision. It is a volumetric deformable recovery method for
states that are already intersecting, especially when a fast solver leaves
preexisting DCD contacts unresolved and CCD can no longer see the earlier
crossing.

The first DART implementation should be internal and tetrahedral-only. It should
consume DART-owned deformable body topology, surface extraction, and collision
candidate records, then produce closest-boundary points, path directions,
diagnostics, and optional recovery constraints or penalty terms for deformable
solvers. PLAN-104/VBD may later reuse this sidecar as a recovery path for
tetrahedral VBD scenes, but SPB should stay owned by PLAN-081 until the shared
mesh/contact contract is stable.

## Method Responsibilities To Audit

| Area                    | SPB responsibility                                                                                                                                                                  | DART starting point                                                                                  | Status  |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- | ------- |
| Topology preconditions  | Tetrahedral volume mesh with boundary surface, face-to-tet adjacency, surface vertices/faces, and inverted-element diagnostics.                                                     | `DeformableBodyOptions` tetrahedra, deterministic boundary-surface extraction, binary serialization. | Planned |
| DCD candidates          | Vertex-tetrahedron and edge-tetrahedron penetration candidates that identify internal query points.                                                                                 | PLAN-081 scene/corpus loading plus deformable contact candidate infrastructure.                      | Planned |
| Boundary candidates     | Boundary-face closest-point candidates, feasible-region culling for vertex/edge/face cases, and ordered BVH traversal.                                                              | Native collision BVH/query foundations; no Embree runtime dependency.                                | Planned |
| Tetrahedral traversal   | Robust path-validity test from candidate boundary point to internal point, including loop recovery, inverted-tet handling, and precise stop reasons.                                | Internal geometry utilities and diagnostics; no public solver/cache type.                            | Planned |
| Recovery output         | Closest boundary point, normal/path direction, path length, source collision row, and failure diagnostics for constraints or penalty forces.                                        | Deformable stage diagnostics and VBD/IPC contact reporting.                                          | Planned |
| Hybrid CCD/DCD behavior | DCD+SPB for preexisting penetrations plus existing CCD/barrier paths for new crossings; no claim of standalone guarantees under high velocity.                                      | PLAN-081 conservative CCD line-search and contact kernels.                                           | Planned |
| Corpus                  | Squishy-ball compression/head-on collision, twisted beam/rods, nested knot, pre-intersected noodle/squishy ball, octopi pile, and long-noodle pile scaled into DART-owned fixtures. | Deformable scene loader, benchmarks, and headless Filament capture path.                             | Planned |

## Implementation Sequence

1. **Source and equation audit** - Pin the arXiv/PDF, supplemental, and reference
   code commit. Map definitions, Theorems 1-2, tetrahedral traversal,
   infeasible-region culling, DCD candidate types, tolerances, and test data to
   DART-owned names and row IDs.
2. **Standalone query slice** - Add internal `detail/deformable_spb` query
   helpers for a static tetrahedral mesh snapshot. Tests cover surface
   extraction, vertex/edge/face candidate classification, zero-length
   boundary/internal coincidences, loop-recovery diagnostics, inverted tets, and
   closest-boundary selection on hand-built meshes.
3. **DCD candidate slice** - Add internal vertex-tet and edge-tet DCD candidate
   generation for deformable volume meshes. Tests distinguish self-collision,
   inter-body collision, adjacency exclusions, and malformed/nonmanifold input
   rejection before SPB is connected to a solver.
4. **Recovery-constraint slice** - Convert SPB query results into internal
   recovery constraints or penalty terms for one deformable solver path. Evidence
   must show decreasing penetration depth, no public API leak, deterministic
   diagnostics, and a clean fallback when the query cannot prove a valid path.
5. **Hybrid CCD/DCD slice** - Pair SPB DCD recovery with existing CCD/barrier
   logic so SPB handles preexisting intersections while CCD handles new
   crossings. Tests must include the failure modes called out by the paper:
   CCD-only missed residual penetrations, DCD-only high-velocity misses, and the
   hybrid path reducing both.
6. **Corpus, benchmark, and visual slice** - Add DART-owned reduced paper scenes
   with invariants, benchmark/profiling JSON, and headless Filament captures.
   The corpus should include at least one pre-intersected recovery scene and one
   large-contact-count scene before any broad robustness claim.

## Acceptance Before Promotion

SPB progress is not promotable from evaluation to an active implementation claim
until DART has:

- a completed paper/supplemental/code matrix with pinned source versions;
- standalone query tests for tetrahedral traversal, feasible-region culling,
  inverted elements, loop recovery, and closest-boundary correctness;
- DCD candidate tests for vertex-tet and edge-tet rows, including self and
  inter-body cases;
- recovery behavior tests showing penetration decreases from pre-intersected
  states without relying on rest-shape shortest paths;
- explicit limitation coverage for codimensional cloth/strands, thin/no-interior
  volumes, non-Euclidean surface geodesics, and high-velocity DCD-only misses;
- benchmark/profiling JSON for query counts, traversed tets, DCD candidates,
  recovery solve cost, and comparison against IPC/CCD-only and any VBD recovery
  baseline used in the same corpus;
- headless Filament evidence for every paper-inspired scene used in the claim;
  and
- `pixi run lint`, `pixi run build`, focused C++ tests, and
  `pixi run check-api-boundaries` green for every implementation slice.

## Non-Goals

- Do not vendor MeshFrame2, CuMatrix, OneTBB, Embree, or the reference test data
  as runtime dependencies.
- Do not expose SPB, the reference repository, Embree, solver registries, ECS
  storage, or backend/resource types through the public C++ or dartpy facade.
- Do not claim SPB handles cloth, strands, shell-only meshes, or arbitrary
  surface geodesics; those are outside the paper's volumetric/tetrahedral scope.
- Do not claim full self-intersection robustness from a standalone query or DCD
  slice before recovery constraints, hybrid CCD/DCD behavior, corpus evidence,
  and visual evidence are in place.
