# Signed Distance Fields and Gradients (Survey)

## Why SDF

Signed distance fields (SDFs) provide continuous distance + gradient queries
useful for trajectory optimization, differentiable simulation, and soft
contacts. SDFs trade memory for fast queries and stable gradients.

## Library Snapshot

| Library  | SDF / voxel support             | Notes                                                        |
| -------- | ------------------------------- | ------------------------------------------------------------ |
| Bullet   | btSdfCollisionShape / btMiniSDF | Includes gradients and CUDA-friendly layouts                 |
| Voxblox  | TSDF + ESDF layers              | Distance/gradient queries with interpolation + batch helpers |
| Parry    | Voxels                          | Voxels are supported; not a full SDF pipeline                |
| FCL/Coal | None                            | Mesh/convex/BVH focus                                        |
| ODE      | None                            | No SDF primitive                                             |
| libccd   | None                            | Convex-only support functions                                |
| OpenGJK  | None                            | Distance-only convex GJK                                     |

## Key References (from awesome-collision-detection)

- Macklin 2020: local optimization for robust SDF contact.
- Koschier 2016: hierarchical hp-adaptive SDFs.
- Voxblox 2017: incremental TSDF to ESDF construction for planning.
- Xu/Barbic 2014: SDFs for polygon soup meshes.

## Gradient Notes (Convex Shapes)

- GJK witness points provide the signed distance gradient almost everywhere:
  `grad = normalize(pointOnObject2 - pointOnObject1)`.
- EPA/MPR penetration direction provides a subgradient when penetrating.
- Gradients are non-unique on edges/vertices; select a stable tie-breaker and
  document it in tests.
- Differentiable collision detection (randomized smoothing) can stabilize
  gradients near feature switches.

## Voxblox Notes (Local Source)

- Data layout: sparse voxel block layers (TSDF/ESDF/occupancy) keyed by hashed block indices.
- Sign convention: distance < 0 denotes occupied/behind the surface; positive is free space.
- Queries: `EsdfMap::getDistanceAtPosition` and `getDistanceAndGradientAtPosition` provide distance + gradient; gradients use central differences at voxel-size offsets with optional trilinear interpolation.
- Observability: interpolated distance/gradient queries require all neighboring voxels observed; missing neighbors return false.
- Batch queries: `batchGetDistanceAtPosition` and `batchGetDistanceAndGradientAtPosition` expose vectorized helpers for planners.

## Integration Targets for DART

- Optional SDF shapes for static environments (voxel or mesh-derived SDF).
- Explicit gradient query API (distance + gradient + witness points).
- SDF benchmark fixtures (resolution sweeps, large-scale scenes).

## Robotics Use Cases (Physics-Based Simulation)

- Fast collision checking for motion planning, MPC, and online replanning using signed distance queries.
- Smooth contact models by using distance-based penalties or barrier functions in simulation/optimization.
- Gradient-based trajectory optimization (e.g., CHOMP/TrajOpt-style) using SDF/ESDF gradients for obstacles.
- Clearance estimation for navigation and footstep planning (robot radius vs ESDF distance).
- Differentiable simulation and learning with distance/gradient signals for contact-aware losses.
- RL training: dense shaping rewards (clearance, progress), safety constraints, and curriculum design based on distance-to-obstacle signals.

## Why SDF/ESDF Over Other Queries

- Dense scalar field yields O(1) distance/gradient queries after preprocessing, unlike repeated mesh/triangle intersection tests.
- Gradients provide smooth, informative signals for optimization and RL, whereas contact-only queries are sparse and discontinuous.
- Supports clearance-aware costs and soft constraints without explicit contact generation or manifold handling.
- ESDF reuse enables batch querying for many samples (particles, rollouts, policies) with predictable latency.
- Voxel grids trade memory for query speed and are resilient to complex, non-convex geometry compared to convex-only pipelines.

## Runtime Comparison Plan

- Accuracy: compare dense-field queries against analytic SDFs (sphere/plane) and a Voxblox ESDF map built from the same grid samples.
- Performance: benchmark query throughput for distance and distance+gradient across dense SDF and Voxblox adapters, using identical query sets and resolutions.
- Reporting: track max/mean absolute distance error, gradient cosine error, and queries/sec; log voxel size, map bounds, and interpolation settings.
- Enable Voxblox comparisons by configuring with `-DDART_EXPERIMENTAL_ESDF_MAP_ROOT=/path/to/voxblox`.
- Optional extensions: compare against Bullet's SDF primitives or Parry voxel queries when those backends are enabled.
