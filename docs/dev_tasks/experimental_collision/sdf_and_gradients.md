# Signed Distance Fields and Gradients (Survey)

## Why SDF

Signed distance fields (SDFs) provide continuous distance + gradient queries
useful for trajectory optimization, differentiable simulation, and soft
contacts. SDFs trade memory for fast queries and stable gradients.

## Library Snapshot

| Library  | SDF / voxel support             | Notes                                         |
| -------- | ------------------------------- | --------------------------------------------- |
| Bullet   | btSdfCollisionShape / btMiniSDF | Includes gradients and CUDA-friendly layouts  |
| Parry    | Voxels                          | Voxels are supported; not a full SDF pipeline |
| FCL/Coal | None                            | Mesh/convex/BVH focus                         |
| ODE      | None                            | No SDF primitive                              |
| libccd   | None                            | Convex-only support functions                 |
| OpenGJK  | None                            | Distance-only convex GJK                      |

## Key References (from awesome-collision-detection)

- Macklin 2020: local optimization for robust SDF contact.
- Koschier 2016: hierarchical hp-adaptive SDFs.
- Voxblox 2016: incremental SDF construction for planning.
- Xu/Barbic 2014: SDFs for polygon soup meshes.

## Gradient Notes (Convex Shapes)

- GJK witness points provide the signed distance gradient almost everywhere:
  `grad = normalize(pointOnObject2 - pointOnObject1)`.
- EPA/MPR penetration direction provides a subgradient when penetrating.
- Gradients are non-unique on edges/vertices; select a stable tie-breaker and
  document it in tests.
- Differentiable collision detection (randomized smoothing) can stabilize
  gradients near feature switches.

## Integration Targets for DART

- Optional SDF shapes for static environments (voxel or mesh-derived SDF).
- Explicit gradient query API (distance + gradient + witness points).
- SDF benchmark fixtures (resolution sweeps, large-scale scenes).
