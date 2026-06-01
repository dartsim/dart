# PLAN-104 OGC Gap Audit

- Operating state: `PLAN-104` in [`../dashboard.md`](../dashboard.md)
- Owner plan:
  [`../104-vertex-block-descent-solver.md`](../104-vertex-block-descent-solver.md)
- Purpose: plan the research and implementation path for Offset Geometric
  Contact (OGC) as a DART-owned codimensional contact model for the experimental
  deformable/VBD stack.

## Source Evidence

- Paper/project: Chen et al., "Offset Geometric Contact," ACM Transactions on
  Graphics 44(4), Article 160, 2025. The project page and PDF describe OGC as a
  penetration-free codimensional contact method that offsets each face along its
  normal direction, computes local per-vertex displacement bounds, and avoids
  continuous collision detection in the solver loop.
- Author project page: records the same SIGGRAPH 2025 publication, the paper
  video, and public links to Gaia and Newton code.
- Gaia reference code: `AnkaChan/Gaia` at HEAD
  `c229692045465a76233f9fba9197fb22bbfb3694` contains the C++ VBD/contact code
  family to audit. Treat it as method evidence only; do not vendor Gaia or expose
  Gaia names through DART's public surface.
- Newton reference code: `newton-physics/newton` at HEAD
  `df43cef24ca61d39f0c7cf0ed542ec1ed8a07580` links OGC to the Warp/Newton VBD
  solver path. Treat it as a comparison and GPU-shape reference, not as a DART
  dependency.

## DART Placement

OGC belongs under PLAN-104 first. It is a contact model for codimensional
deformable primitives and is presented with VBD as the fast solver path. It
should reuse the PLAN-081 shared deformable contact kernels and broad-phase
foundations where possible, then compare against IPC-class barriers before any
claim that it supersedes part of PLAN-081. It is not a PLAN-082 event-level
simultaneous-impact operator and should not be framed as a rigid-impact
replacement.

The public deformable facade remains algorithm-neutral. Until the audit and
implementation gates below pass, OGC is an internal method family: no public
`OgcSolver`, no solver registry, no Gaia/Newton/Warp dependency, and no device or
backend type in public C++ or dartpy signatures.

## Method Responsibilities To Audit

| Area                | OGC responsibility                                                                                                                                                      | DART starting point                                                                        | Status  |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ | ------- |
| Offset geometry     | Intersection-aware offset geometry for triangle faces and edge/vertex subfaces, with normals that keep contact forces orthogonal to the contacted face.                 | PLAN-081 distance/derivative kernels and PLAN-104 surface self-contact penalties.          | Planned |
| Contact detection   | Vertex-facet and edge-edge queries over non-offset primitives, duplicate suppression, adjacent primitive filtering, and contact set construction.                       | Native collision BVH/query utilities plus deformable surface topology.                     | Planned |
| Conservative bounds | Per-vertex displacement bounds from local distance evidence so penetration-free steps do not require CCD every solver iteration.                                        | Existing VBD iteration/update loop and runtime diagnostics.                                | Planned |
| Contact energy      | Normal contact energy plus lagged friction terms, with force/Hessian accumulation usable by per-vertex VBD blocks.                                                      | VBD force/Hessian block kernels and current lagged surface contact terms.                  | Planned |
| Bound truncation    | Initial guess and per-iteration displacement truncation, with contact refresh controlled by an exceeded-bound threshold.                                                | VBD solver loop and stage stats.                                                           | Planned |
| CPU/GPU shape       | Local operations, deterministic reductions, and fixed-budget iteration evidence before any GPU claim.                                                                   | PLAN-030 compute boundary and PLAN-104 CPU/CUDA benchmark harnesses.                       | Planned |
| Corpus              | Cloth twisting, knot tightening, yarn/rod stress, cloth-on-body, T-shirt manipulation, and large layered cloth, scaled into DART-owned tests, examples, and benchmarks. | Existing deformable/VBD demos, headless Filament capture path, benchmark JSON conventions. | Planned |

## Implementation Sequence

1. **Source and equation audit** - Pin the exact paper PDF, Gaia commit, and
   Newton commit. Map OGC equations, algorithms, and code paths to DART-owned
   names, data structures, and tests. Record which rows are paper-only,
   Gaia-backed, Newton-backed, or missing.
2. **CPU detector slice** - Add internal `detail/deformable_ogc` geometry and
   contact-set construction for vertex-facet and edge-edge rows on static mesh
   snapshots. This slice does not affect `World::step()`. Tests prove adjacency
   filtering, duplicate suppression, closest-subface classification, and
   deterministic contact sets on small hand-built meshes.
3. **Conservative-bound slice** - Compute per-vertex bounds from the detected
   contacts and nearest distances, then add standalone truncation tests. Evidence
   must show that a penetration-free state remains penetration-free under bounded
   displacements and that high-velocity sparse-contact cases are reported as an
   OGC weakness rather than hidden by broad claims.
4. **VBD integration slice** - Wire OGC contact energy and lagged friction into
   the internal VBD contact accumulation path behind DART-owned policy names.
   Tests compare force/Hessian finite differences, block energy decrease,
   penetration-free repeated steps, and behavior against the existing IPC-style
   contact kernels on matched scenes.
5. **Corpus and visual slice** - Add DART-owned reduced versions of the paper
   scenes: twisted cloth, knot pull, layered cloth, yarn/rod pull-through, and
   cloth-on-body. Each scene needs invariants, a focused test or benchmark row,
   profiling JSON, and long-horizon headless Filament captures before being used
   as acceptance evidence.
6. **GPU/private compute slice** - Only after CPU correctness and corpus gates
   pass, map the local operations to DART's private compute boundary. Claims
   require CPU and GPU benchmark packets on the same corpus, deterministic
   reduction policy, and proof that no runtime GPU dependency leaks into the
   default build.

## Acceptance Before Promotion

OGC progress is not promotable from evaluation to an active implementation claim
until DART has:

- a completed source/code matrix for the paper PDF, Gaia, and Newton paths;
- internal tests for vertex-facet and edge-edge OGC contact construction,
  adjacent/duplicate filtering, conservative bounds, truncation, and refresh
  thresholds;
- force and Hessian finite-difference coverage for the normal and friction terms
  used by VBD;
- penetration-free repeated-step evidence from initially valid codimensional
  scenes, including at least one scene where IPC's small contact radius or CCD
  line-search cost is the comparison target;
- explicit tests or documentation for OGC limitations: contact-force
  discontinuities and high-velocity sparse-contact cases where CCD-aware IPC can
  take a larger step;
- benchmark/profiling JSON comparing current VBD contact, IPC-style barriers,
  OGC CPU, and any private GPU path on the same DART-owned corpus;
- headless Filament evidence for the paper-inspired scenes used in the claim;
  and
- `pixi run lint`, `pixi run build`, focused C++ tests, and
  `pixi run check-api-boundaries` green for every implementation slice.

## Non-Goals

- Do not vendor Gaia, Newton, Warp, or paper assets as runtime dependencies.
- Do not expose OGC, Gaia, Newton, Warp, solver registries, ECS storage, or
  compute backend types through the public facade.
- Do not use OGC to replace rigid IPC, simultaneous-impact evaluation, or
  general rigid contact before a DART-owned comparison corpus proves that scope.
- Do not claim full paper parity from a detector, bound, or single-scene slice.
