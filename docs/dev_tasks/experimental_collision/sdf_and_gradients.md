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

- Data layout: sparse voxel block `Layer` keyed by hashed block indices; each `Block` stores a dense voxel array plus per-block update flags and origin.
- TSDF pipeline: `TsdfVoxel` stores distance/weight/color; simple/merged/fast integrators raycast point clouds with truncation, range limits, and optional clearing or weight drop-off.
- ESDF pipeline: `EsdfVoxel` tracks distance, observed/fixed flags, queue state, parent direction, and hallucinated status; raise/lower wavefront updates via a bucketed priority queue with fixed-band and min-diff thresholds; full vs quasi-Euclidean distance uses 6/18/26 neighbor offsets.
- Queries: `Interpolator` supports trilinear distance/weight/gradient queries plus adaptive one-sided gradients when neighbors are missing; `EsdfMap` exposes distance + gradient accessors.
- Observability and batch: interpolated queries require all neighbors observed; batch distance/gradient helpers target Python/planning use.
- Build notes: ROS/catkin-centric build with glog/protobuf dependencies.

## DART Experimental Dense TSDF/ESDF Notes

- `DenseTsdfField` stores truncated distances, weights, and observed flags on a dense grid for small to mid-sized scenes.
- `DenseEsdfField` builds from `DenseTsdfField` using surface seeds (distance band or sign changes) and a Dijkstra-style wavefront over 6 or 26 neighbors.
- Build is batch-only (no incremental raise/lower), so updates are best done offline or at low frequency.
- Distances are quasi-Euclidean on the voxel grid and bounded by `maxDistance`; sign is inherited from the TSDF.

## Integration Targets for DART

- Optional SDF/ESDF shapes for static environments (voxel or mesh-derived fields).
- TSDF to ESDF conversion API for building navigable clearance fields.
- Explicit gradient query API (distance + gradient + witness points).
- SDF benchmark fixtures (resolution sweeps, large-scale scenes).

## Placement in DART (Core vs Extension)

- SDF/ESDF are a strong fit for `dart/collision/experimental` where we can iterate on data layout and query APIs without locking core behavior.
- Keep core collision focused on analytic/BVH-backed shapes; treat SDF/ESDF as an opt-in module for robotics environments and planner-centric workloads.
- Use SDF/ESDF for static or slowly changing geometry; dynamic articulated bodies still need exact contact generation and manifold support.

## CCD Considerations

- SDF/ESDF can support conservative advancement or ray-marching along trajectories, but they do not replace exact CCD for convex casts or contact manifolds.
- Use SDF-based queries to bound step sizes or provide fast clearance checks; fall back to analytic CCD for precise time-of-impact.

## Robotics Use Cases (Physics-Based Simulation)

- Fast clearance checks for planning, MPC, and online replanning using signed distance queries.
- Smooth contact models using distance-based penalties or barrier functions in simulation/optimization.
- Gradient-based trajectory optimization (CHOMP/TrajOpt-style) from SDF/ESDF gradients.
- Navigation and footstep clearance estimation (robot radius vs ESDF distance).
- Differentiable simulation and learning with distance/gradient signals for contact-aware losses.
- RL training: dense shaping rewards, safety constraints, and curriculum design tied to distance-to-obstacle signals.

## Why SDF/ESDF Over Other Queries

- Dense scalar field yields O(1) distance/gradient queries after preprocessing, unlike repeated mesh/triangle intersection tests.
- Gradients provide smooth, informative signals for optimization and RL, whereas contact-only queries are sparse and discontinuous.
- Supports clearance-aware costs and soft constraints without explicit contact generation or manifold handling.
- ESDF reuse enables batch querying for many samples (particles, rollouts, policies) with predictable latency.
- Voxel grids trade memory for query speed and remain robust on complex, non-convex environments where convex-only pipelines struggle.

## Tradeoffs vs Other Data Structures

- Memory scales with voxel resolution; dense grids are fast but expensive compared to BVHs or adaptive meshes.
- Resolution limits accuracy, especially for thin features and high-curvature surfaces.
- Dynamic scenes require frequent rebuilds or sparse/incremental updates; dense batch builds are best for static worlds.

## Runtime Comparison Plan

- Accuracy: compare dense-field queries against analytic SDFs (sphere/plane) and optional Voxblox ESDF results built on the same grid samples (tests/bench only).
- Performance: benchmark dense SDF/ESDF query throughput and ESDF build time; optionally compare to Voxblox using identical query sets and resolutions.
- Reporting: track max/mean absolute distance error, gradient cosine error, and queries/sec; log voxel size, map bounds, and interpolation settings.
- Keep Voxblox comparisons limited to tests/benchmarks so the collision module has no external dependency (enable with `-DDART_EXPERIMENTAL_VOXBLOX_ROOT=/path/to/voxblox`).
- Optional extensions: compare against Bullet SDF primitives or Parry voxel queries when those backends are enabled.
